#!/usr/bin/env python3

import urllib.request
import json
import re

ZONES = ['shelf_A', 'shelf_B', 'shelf_C', 'loading_dock', 'sorting_area', 'charging_station']

TEST_CASES = [
    # 1. New Task Verbs (Out of template set)
    {
        "id": "V1",
        "category": "New Verbs",
        "instruction": "hoist 2 pallets from shelf 1 and deposit them at offloading dock",
        "description": "Uses 'hoist' and 'deposit' instead of 'pick'/'place'/'transfer'."
    },
    {
        "id": "V2",
        "category": "New Verbs",
        "instruction": "replenish sorting area with 3 boxes from shelf 2",
        "description": "Uses 'replenish ... with ... from' syntax."
    },
    {
        "id": "V3",
        "category": "New Verbs",
        "instruction": "store the cargo from shelf 3 in sorting area",
        "description": "Uses 'store ... in ...' syntax."
    },
    {
        "id": "V4",
        "category": "New Verbs",
        "instruction": "evacuate shelf A",
        "description": "Uses 'evacuate' (similar to clear)."
    },
    
    # 2. Compound Conditions & Dependencies
    {
        "id": "C1",
        "category": "Compound Conditions",
        "instruction": "retrieve 2 pallets from shelf 1 to loading dock, and then go charge the robot",
        "description": "Sequential task combining retrieval and charging."
    },
    {
        "id": "C2",
        "category": "Compound Conditions",
        "instruction": "clear shelf B, but if battery is below 20% go to charging station instead",
        "description": "Conditional instruction based on state parameters."
    },
    {
        "id": "C3",
        "category": "Compound Conditions",
        "instruction": "if robot 0 is at shelf C, move it to sorting area",
        "description": "State dependency check in instruction."
    },
    
    # 3. Ambiguous References
    {
        "id": "A1",
        "category": "Ambiguous References",
        "instruction": "move it to the dock",
        "description": "Uses pronoun 'it' and vague destination 'dock'."
    },
    {
        "id": "A2",
        "category": "Ambiguous References",
        "instruction": "take the box from there to the sorting area",
        "description": "Uses vague pronoun 'there'."
    },
    {
        "id": "A3",
        "category": "Ambiguous References",
        "instruction": "navigate to the nearest shelf",
        "description": "Uses relative/proximity target 'nearest shelf'."
    }
]

def query_ollama(text):
    # Apply standard normalization rules first
    normalized = re.sub(r"\b(shelp|shlef|shefl|shelves|shelfs)\b", "shelf", text, flags=re.IGNORECASE)
    normalized = re.sub(r"\bshelf\s*1\b", "shelf_A", normalized, flags=re.IGNORECASE)
    normalized = re.sub(r"\bshelf\s*2\b", "shelf_B", normalized, flags=re.IGNORECASE)
    normalized = re.sub(r"\bshelf\s*3\b", "shelf_C", normalized, flags=re.IGNORECASE)
    normalized = re.sub(r"\b(offloading bay|loading bay|loading area|delivery dock|delivery bay|shipping bay|offloading dock|offload bay|offload dock)\b", "loading_dock", normalized, flags=re.IGNORECASE)
    normalized = re.sub(r"\b(sorting bay|sorting dock)\b", "sorting_area", normalized, flags=re.IGNORECASE)
    
    prompt = (
        "You are a warehouse task planner. Translate the following natural language instruction into a sequence of structured tasks.\n"
        f"Valid zone names: {ZONES}\n"
        "Valid task types: \"pick\", \"place\", \"charge\", \"go_to\"\n\n"
        "Rules:\n"
        "1. \"transfer\", \"move\", \"transport\", \"deliver\", \"carry\", \"bring\", \"take\" from A to B:\n"
        "   - task_type: \"pick\" at A (priority: 1)\n"
        "   - task_type: \"place\" at B (priority: 1)\n"
        "2. \"clear\", \"empty\", \"cleanup\" A:\n"
        "   - task_type: \"pick\" at A (priority: 2)\n"
        "   - task_type: \"place\" at sorting_area (priority: 2)\n"
        "3. \"charge\" robot:\n"
        "   - task_type: \"charge\" at charging_station (priority: 3)\n"
        "4. \"go to\", \"navigate to\", \"visit\", \"head to\" A:\n"
        "   - task_type: \"go_to\" at A (priority: 0)\n"
        "5. If a source is specified (e.g., from shelf_A) but no destination is specified:\n"
        "   - task_type: \"pick\" at the source (priority: 1)\n"
        "   - task_type: \"place\" at loading_dock (priority: 1)\n\n"
        "Examples:\n"
        "Instruction: \"transfer from shelf_A to loading_dock\"\n"
        "Output: [{\"task_type\": \"pick\", \"target_zone\": \"shelf_A\", \"priority\": 1}, {\"task_type\": \"place\", \"target_zone\": \"loading_dock\", \"priority\": 1}]\n\n"
        f"Instruction: \"{normalized}\"\n"
        "Output ONLY raw JSON array, no extra words or markdown:"
    )
    
    url = "http://localhost:11434/api/generate"
    data = {
        "model": "qwen2:7b-instruct",
        "prompt": prompt,
        "stream": False,
        "options": {"temperature": 0.0}
    }
    
    try:
        req = urllib.request.Request(
            url,
            data=json.dumps(data).encode('utf-8'),
            headers={'Content-Type': 'application/json'}
        )
        with urllib.request.urlopen(req, timeout=8.0) as response:
            res_body = json.loads(response.read().decode('utf-8'))
            response_text = res_body.get('response', '').strip()
            
        if response_text.startswith("```json"):
            response_text = response_text[7:]
        if response_text.endswith("```"):
            response_text = response_text[:-3]
        response_text = response_text.strip()
        
        return json.loads(response_text), normalized
    except Exception as e:
        return f"Error: {e}", normalized

def main():
    print("=================================================================")
    print("                 FLEETLANG NLU PARSER STRESS TEST                ")
    print("=================================================================\n")
    
    report_lines = [
        "# FleetLang NLU Parser Stress Test Report",
        "",
        "This report evaluates the out-of-distribution (OOD) capability of the FleetLang NLU parser running `qwen2:7b-instruct`. It runs test cases across new verbs, compound conditions, and ambiguous references to identify failure modes.",
        "",
        "| ID | Category | Original Prompt | Normalized Prompt | Parsed Result | Evaluation & Classification |",
        "| :--- | :--- | :--- | :--- | :--- | :--- |"
    ]
    
    for case in TEST_CASES:
        cid = case["id"]
        cat = case["category"]
        inst = case["instruction"]
        desc = case["description"]
        
        print(f"[{cid}] Running Category '{cat}': '{inst}'")
        res, norm = query_ollama(inst)
        
        # Classify result
        classification = "Unknown"
        if isinstance(res, str) and res.startswith("Error:"):
            classification = "💥 Complete Failure (JSON Syntax/HTTP Error)"
        elif isinstance(res, list):
            if len(res) == 0:
                classification = "⚠️ Empty Output (Undetected task)"
            else:
                # Check for validity
                all_valid = True
                for task in res:
                    tt = task.get("task_type")
                    tz = task.get("target_zone")
                    if tt not in ["pick", "place", "charge", "go_to"] or tz not in ZONES:
                        all_valid = False
                        break
                if all_valid:
                    classification = "✅ Success (Grounds to Valid Schema)"
                else:
                    classification = "❌ Hallucination (Contains invalid action/zone)"
        
        res_str = json.dumps(res) if not isinstance(res, str) else res
        print(f"  > Normalized: {norm}")
        print(f"  > Result:     {res_str}")
        print(f"  > Class:      {classification}\n")
        
        report_lines.append(f"| {cid} | {cat} | *\"{inst}\"* | *\"{norm}\"* | `{res_str}` | {classification} |")
        
    # Write report file
    with open("/home/hannan/workspace/FleetLang/results/parser_stress_test_report.md", "w") as f:
        f.write("\n".join(report_lines) + "\n")
    print("Stress test report successfully saved to results/parser_stress_test_report.md")

if __name__ == "__main__":
    main()
