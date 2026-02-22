---
sidebar_position: 2
---

# LLM Cognitive Planning

## Overview

Large Language Models (LLMs) can decompose high-level instructions into executable robot actions:

**Input:** "Pick up the red ball and place it on the table"

**LLM Output:**
```json
{
  "steps": [
    {"action": "navigate", "target": "red_ball", "preconditions": []},
    {"action": "detect", "object": "red_ball", "preconditions": ["near(red_ball)"]},
    {"action": "reach", "target": "red_ball", "preconditions": ["visible(red_ball)"]},
    {"action": "grasp", "object": "red_ball", "preconditions": ["reached(red_ball)"]},
    {"action": "navigate", "target": "table", "preconditions": ["holding(red_ball)"]},
    {"action": "place", "object": "red_ball", "location": "table", "preconditions": ["near(table)"]}
  ]
}
```

## Using OpenAI API

```python
from openai import OpenAI
import json

client = OpenAI(api_key="your-api-key")

def plan_task(natural_language_instruction):
    prompt = f"""
    You are a robot planning assistant. Decompose the instruction into atomic robot actions.
    
    Available actions:
    - navigate(target): Move to a location
    - detect(object): Identify an object
    - reach(target): Extend arm to target
    - grasp(object): Close gripper on object
    - place(object, location): Release object at location
    - return_home(): Return to charging station
    
    Instruction: {natural_language_instruction}
    
    Output a JSON array of steps with action, parameters, and preconditions.
    """
    
    response = client.chat.completions.create(
        model="gpt-4o",
        messages=[
            {"role": "system", "content": "You are a robot planning assistant."},
            {"role": "user", "content": prompt}
        ],
        response_format={"type": "json_object"}
    )
    
    plan = json.loads(response.choices[0].message.content)
    return plan

# Usage
plan = plan_task("Bring me the water bottle from the kitchen")
print(json.dumps(plan, indent=2))
```

## Using Local LLMs (Llama)

```python
from transformers import AutoModelForCausalLM, AutoTokenizer
import torch

class LocalLLMPlanner:
    def __init__(self, model_name="meta-llama/Llama-2-7b-chat-hf"):
        self.tokenizer = AutoTokenizer.from_pretrained(model_name)
        self.model = AutoModelForCausalLM.from_pretrained(
            model_name,
            torch_dtype=torch.float16,
            device_map="auto"
        )
    
    def plan(self, instruction):
        prompt = f"""[INST] You are a robot planning assistant.
        Decompose this instruction into robot actions: {instruction}
        Output JSON format only. [/INST]"""
        
        inputs = self.tokenizer(prompt, return_tensors="pt").to(self.model.device)
        outputs = self.model.generate(
            **inputs,
            max_new_tokens=500,
            temperature=0.1,
            do_sample=True
        )
        
        response = self.tokenizer.decode(outputs[0], skip_special_tokens=True)
        return self.extract_json(response)
    
    def extract_json(self, text):
        # Extract JSON from response
        import re
        json_match = re.search(r'\{.*\}', text, re.DOTALL)
        if json_match:
            return json.loads(json_match.group())
        return None

# Usage
planner = LocalLLMPlanner()
plan = planner.plan("Clean up the toys in the living room")
```

## Plan Validation

```python
def validate_plan(plan, world_state):
    """Validate that a plan is executable given current world state"""
    for i, step in enumerate(plan["steps"]):
        preconditions = step.get("preconditions", [])
        for precondition in preconditions:
            if not check_precondition(precondition, world_state):
                return False, f"Step {i}: Precondition '{precondition}' not met"
    return True, "Plan is valid"

def check_precondition(precondition, world_state):
    """Check if a precondition is satisfied"""
    # Parse precondition
    if precondition.startswith("near("):
        target = precondition[5:-1]
        return world_state.get("robot_near", []) == target
    elif precondition.startswith("visible("):
        obj = precondition[8:-1]
        return obj in world_state.get("visible_objects", [])
    elif precondition.startswith("holding("):
        obj = precondition[8:-1]
        return world_state.get("holding") == obj
    return False
```

## Lab Exercise 4.2

Build an LLM planner:
1. Set up OpenAI API or local LLM
2. Define robot action vocabulary
3. Create prompt templates
4. Test with various instructions
5. Validate generated plans

## Next Steps

Learn about grounding language to ROS actions.
