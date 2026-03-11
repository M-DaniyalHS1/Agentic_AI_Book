"""
Script to rebuild/reindex all textbook content embeddings in Qdrant

Usage:
    python backend/scripts/rebuild_embeddings.py

This script:
1. Reads all Markdown files from frontend/docs/
2. Parses them into sections
3. Generates embeddings
4. Indexes them in Qdrant
"""
import os
import sys
import re
from pathlib import Path
from dotenv import load_dotenv

# Add backend to path
sys.path.insert(0, str(Path(__file__).parent.parent))

load_dotenv()

from src.services.qdrant_client import get_qdrant_client, COLLECTION_NAME, init_qdrant_collection
from src.utils.embedding_utils import generate_embedding_sync
from qdrant_client.models import PointStruct

# Configuration
DOCS_DIR = Path(__file__).parent.parent.parent / "frontend" / "docs"


def parse_markdown_file(file_path: Path) -> list:
    """
    Parse a Markdown file into sections

    Returns list of dicts with: title, slug, content
    """
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Extract frontmatter
    frontmatter = {}
    if content.startswith('---'):
        match = re.search(r'---\n(.*?)\n---', content, re.DOTALL)
        if match:
            fm_text = match.group(1)
            for line in fm_text.split('\n'):
                if ':' in line:
                    key, value = line.split(':', 1)
                    frontmatter[key.strip()] = value.strip()
            content = content[match.end():].strip()

    # Get title from frontmatter or first heading
    title = frontmatter.get('title', '')
    if not title:
        heading_match = re.search(r'^#\s+(.+)$', content, re.MULTILINE)
        if heading_match:
            title = heading_match.group(1)

    # Split by headings (## or ###)
    sections = []
    current_section = {
        'title': title or 'Introduction',
        'slug': file_path.stem,
        'content': ''
    }

    # Split by ## headings
    heading_pattern = re.compile(r'^##\s+(.+)$', re.MULTILINE)
    parts = heading_pattern.split(content)

    if len(parts) == 1:
        # No ## headings, treat entire content as one section
        current_section['content'] = content
        sections.append(current_section)
    else:
        # First part is content before any ## heading
        if parts[0].strip():
            current_section['content'] = parts[0].strip()
            sections.append(current_section)

        # Process remaining parts (heading, content, heading, content, ...)
        for i in range(1, len(parts), 2):
            if i + 1 < len(parts):
                section_title = parts[i].strip()
                section_content = parts[i + 1].strip()
                sections.append({
                    'title': section_title,
                    'slug': section_title.lower().replace(' ', '-').replace('.', ''),
                    'content': f"## {section_title}\n\n{section_content}"
                })

    return sections


def get_module_chapter_from_path(file_path: Path) -> tuple:
    """
    Extract module and chapter slugs from file path

    Example: frontend/docs/module-1/ros2-architecture.md
    Returns: ('module-1', 'ros2-architecture')
    """
    try:
        rel_path = file_path.relative_to(DOCS_DIR)
        parts = rel_path.parts

        if len(parts) >= 2:
            return parts[0], parts[1].replace('.md', '')
        elif len(parts) == 1:
            return 'root', parts[0].replace('.md', '')
        else:
            return 'unknown', 'unknown'
    except ValueError:
        return 'unknown', 'unknown'


def index_all_content():
    """Index all textbook content in Qdrant"""
    print(f"📚 Indexing textbook content from: {DOCS_DIR}")

    if not DOCS_DIR.exists():
        print(f"❌ Docs directory not found: {DOCS_DIR}")
        return False

    # Initialize Qdrant
    client = get_qdrant_client()
    init_qdrant_collection(client)

    # Find all markdown files
    markdown_files = list(DOCS_DIR.rglob("*.md"))
    print(f"📄 Found {len(markdown_files)} markdown files")

    indexed_count = 0
    error_count = 0

    for file_path in markdown_files:
        try:
            module_slug, chapter_slug = get_module_chapter_from_path(file_path)

            # Skip index.md files at root level
            if file_path.name == 'index.md' and file_path.parent == DOCS_DIR:
                continue

            print(f"\n📖 Processing: {file_path.relative_to(DOCS_DIR)}")

            # Parse sections
            sections = parse_markdown_file(file_path)

            for section in sections:
                if not section['content'].strip():
                    continue

                # Generate embedding (synchronous version)
                embedding = generate_embedding_sync(section['content'])

                # Create unique ID
                point_id = hash(f"{module_slug}:{chapter_slug}:{section['slug']}") % (2**63)

                # Create payload
                payload = {
                    "module_slug": module_slug,
                    "chapter_slug": chapter_slug,
                    "section_slug": section['slug'],
                    "section_title": section['title'],
                    "content": section['content'],
                    "file_path": str(file_path.relative_to(DOCS_DIR))
                }

                # Create point
                point = PointStruct(
                    id=point_id,
                    vector=embedding,
                    payload=payload
                )

                # Upsert to Qdrant
                client.upsert(
                    collection_name=COLLECTION_NAME,
                    points=[point]
                )

                indexed_count += 1
                print(f"  ✓ Indexed: {section['title']}")

        except Exception as e:
            error_count += 1
            print(f"  ❌ Error processing {file_path}: {e}")

    print(f"\n{'='*50}")
    print(f"✅ Indexing complete!")
    print(f"   Indexed: {indexed_count} sections")
    print(f"   Errors: {error_count}")
    print(f"{'='*50}")

    return indexed_count > 0


if __name__ == "__main__":
    success = index_all_content()
    sys.exit(0 if success else 1)
