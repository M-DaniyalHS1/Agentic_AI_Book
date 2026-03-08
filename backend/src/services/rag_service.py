"""
RAG (Retrieval-Augmented Generation) Service
Core service for retrieving relevant textbook content and generating AI responses
Uses FREE providers: Groq (LLM) + HuggingFace (Embeddings)
"""
import os
import time
from typing import List, Dict, Optional, Tuple
from dotenv import load_dotenv

from .qdrant_client import get_qdrant_client, COLLECTION_NAME, EMBEDDING_DIMENSION
from ..utils.embedding_utils import generate_embedding, generate_embeddings, get_embedding_dimension
from ..utils.llm_utils import generate_response

load_dotenv()

# RAG Configuration
TOP_K_RESULTS = 5
SIMILARITY_THRESHOLD = 0.7
MAX_CONTEXT_TOKENS = 4000
MAX_RESPONSE_TOKENS = 500


class RAGService:
    """
    RAG Service for retrieving textbook content and generating AI tutor responses
    """

    def __init__(self):
        self.qdrant_client = get_qdrant_client()
        self.collection_name = COLLECTION_NAME

    def search_relevant_content(
        self,
        query: str,
        top_k: int = TOP_K_RESULTS,
        similarity_threshold: float = SIMILARITY_THRESHOLD,
        module_filter: Optional[str] = None,
        chapter_filter: Optional[str] = None
    ) -> List[Dict]:
        """
        Search for relevant textbook content using vector similarity

        Args:
            query: The search query
            top_k: Number of results to return
            similarity_threshold: Minimum similarity score
            module_filter: Optional module slug filter
            chapter_filter: Optional chapter slug filter

        Returns:
            List of relevant content with metadata
        """
        # Generate query embedding
        query_embedding = generate_embedding(query)

        # Build filter
        query_filter = None
        if module_filter or chapter_filter:
            must_conditions = []
            if module_filter:
                must_conditions.append({
                    "key": "module_slug",
                    "match": {"value": module_filter}
                })
            if chapter_filter:
                must_conditions.append({
                    "key": "chapter_slug",
                    "match": {"value": chapter_filter}
                })
            query_filter = {"must": must_conditions}

        # Search in Qdrant
        try:
            results = self.qdrant_client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                query_filter=query_filter,
                limit=top_k,
                score_threshold=similarity_threshold
            )

            # Format results
            formatted_results = []
            for result in results:
                payload = result.payload
                formatted_results.append({
                    "id": result.id,
                    "module_slug": payload.get("module_slug"),
                    "chapter_slug": payload.get("chapter_slug"),
                    "section_slug": payload.get("section_slug"),
                    "section_title": payload.get("section_title"),
                    "content": payload.get("content"),
                    "similarity_score": result.score
                })

            return formatted_results

        except Exception as e:
            print(f"Error searching Qdrant: {e}")
            return []

    def build_context(self, results: List[Dict], max_tokens: int = MAX_CONTEXT_TOKENS) -> str:
        """
        Build context string from search results for LLM prompt

        Args:
            results: List of search results
            max_tokens: Maximum tokens for context

        Returns:
            Formatted context string
        """
        if not results:
            return "No relevant content found in the textbook."

        context_parts = []
        current_length = 0

        for result in results:
            section = f"""
---
Module: {result['module_slug']}
Chapter: {result['chapter_slug']}
Section: {result['section_title']}

{result['content'][:500]}  # Truncate long sections
"""
            # Simple token estimation (~4 chars per token)
            estimated_tokens = len(section) / 4
            if current_length + estimated_tokens > max_tokens:
                break

            context_parts.append(section)
            current_length += estimated_tokens

        return "\n".join(context_parts)

    async def generate_response(
        self,
        query: str,
        context: str,
        selected_text: Optional[str] = None
    ) -> Tuple[str, float]:
        """
        Generate AI response using LLM with retrieved context (FREE: Groq/HuggingFace)

        Args:
            query: User's question
            context: Retrieved textbook content
            selected_text: Optional selected text for explanation

        Returns:
            Tuple of (response_text, confidence_score)
        """
        # Build prompt
        if selected_text:
            system_prompt = f"""You are an AI tutor for Physical AI & Humanoid Robotics.
Your task is to explain the selected textbook content to students.

SELECTED TEXT: "{selected_text}"

RELEVANT TEXTBOOK CONTEXT:
{context}

INSTRUCTIONS:
- Explain the selected text using the context provided
- Use clear, educational language appropriate for students
- If the context doesn't help explain the selected text, say so
- Keep your explanation focused and concise (max 300 words)
"""
        else:
            system_prompt = f"""You are an AI tutor for Physical AI & Humanoid Robotics.
Answer questions based ONLY on the textbook content provided below.

TEXTBOOK CONTEXT:
{context}

QUESTION: {query}

INSTRUCTIONS:
- Answer based solely on the provided context
- If the context doesn't contain the answer, say "This is not covered in the book yet. Please check the table of contents for available topics."
- Use clear, educational language
- Cite specific chapters and sections when possible
- Keep responses concise (max 400 words)
"""

        # Call LLM using free providers (Groq, HuggingFace)
        try:
            messages = [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": query}
            ]
            
            answer = await generate_response(
                messages=messages,
                max_tokens=MAX_RESPONSE_TOKENS,
                temperature=0.7
            )
            
            # Estimate confidence based on response quality
            confidence = 0.8 if len(answer) > 50 else 0.5
            return answer, confidence

        except Exception as e:
            print(f"Error generating LLM response: {e}")
            return self._generate_fallback_response(query, context, selected_text)

    def _generate_fallback_response(
        self,
        query: str,
        context: str,
        selected_text: Optional[str] = None
    ) -> Tuple[str, float]:
        """Generate a fallback response when LLM is unavailable"""
        if selected_text:
            return (
                f"I can help explain: \"{selected_text[:100]}...\"\n\n"
                f"Based on the textbook content, this relates to concepts in the retrieved sections. "
                f"Please refer to the cited sections above for detailed information.",
                0.6
            )
        else:
            return (
                "This is not covered in the book yet. Please check the table of contents for available topics, "
                "or try asking about ROS 2, Digital Twins, AI-Robot Brain, or Vision-Language-Action systems.",
                0.5
            )

    async def process_query(
        self,
        query: str,
        selected_text: Optional[str] = None,
        module_filter: Optional[str] = None,
        chapter_filter: Optional[str] = None
    ) -> Dict:
        """
        Process a complete user query through the RAG pipeline

        Args:
            query: User's question
            selected_text: Optional selected text for explanation
            module_filter: Optional module filter
            chapter_filter: Optional chapter filter

        Returns:
            Dictionary with answer, citations, confidence, and fallback flag
        """
        start_time = time.time()

        # Use selected text as query if available
        search_query = selected_text if selected_text else query

        # Search for relevant content
        results = self.search_relevant_content(
            query=search_query,
            module_filter=module_filter,
            chapter_filter=chapter_filter
        )

        # Build context
        context = self.build_context(results)

        # Generate response (async)
        answer, confidence = await self.generate_response(
            query=query,
            context=context,
            selected_text=selected_text
        )

        # Build citations
        citations = []
        for i, result in enumerate(results):
            citations.append({
                "module_slug": result["module_slug"],
                "chapter_slug": result["chapter_slug"],
                "section_slug": result["section_slug"],
                "section_title": result["section_title"],
                "content_snippet": result["content"][:200] + "...",
                "similarity_score": result["similarity_score"]
            })

        response_time_ms = int((time.time() - start_time) * 1000)
        is_fallback = "not covered in the book" in answer.lower()

        return {
            "answer": answer,
            "citations": citations,
            "confidence": confidence,
            "is_fallback": is_fallback,
            "response_time_ms": response_time_ms
        }

    def index_content(
        self,
        module_slug: str,
        chapter_slug: str,
        section_slug: str,
        section_title: str,
        content: str
    ) -> bool:
        """
        Index a section of textbook content in Qdrant

        Args:
            module_slug: Module identifier
            chapter_slug: Chapter identifier
            section_slug: Section identifier
            section_title: Section title
            content: Section content

        Returns:
            True if successful
        """
        try:
            # Generate embedding
            embedding = generate_embedding(content)

            # Create payload
            payload = {
                "module_slug": module_slug,
                "chapter_slug": chapter_slug,
                "section_slug": section_slug,
                "section_title": section_title,
                "content": content
            }

            # Create point
            point = PointStruct(
                id=hash(f"{module_slug}:{chapter_slug}:{section_slug}") % (2**63),
                vector=embedding,
                payload=payload
            )

            # Upsert to Qdrant
            self.qdrant_client.upsert(
                collection_name=self.collection_name,
                points=[point]
            )

            return True

        except Exception as e:
            print(f"Error indexing content: {e}")
            return False


# Singleton instance
_rag_service_instance = None


def get_rag_service() -> RAGService:
    """Get or create RAG service singleton"""
    global _rag_service_instance
    if _rag_service_instance is None:
        _rag_service_instance = RAGService()
    return _rag_service_instance
