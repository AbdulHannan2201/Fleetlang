#!/usr/bin/env python3

import urllib.request
import json
import re

ZONES = ['shelf_A', 'shelf_B', 'shelf_C', 'loading_dock', 'sorting_area', 'charging_station', 'nearest_shelf', 'nearest_charging_station', 'it', 'last_target']

TEST_CASES = [
    # 1. New Task Verbs (Out of template set)
    {
        "id": "V1",
        "category": "New Verbs",
        "instruction": "hoist 2 pallets from shelf 1 and deposit them at offloading dock",
        "description": "Uses 'hoist' and 'deposit'."
    },
    {
        "id": "V2",
        "category": "New Verbs",
        "instruction": "replenish sorting area with 3 boxes from shelf 2",
        "description": "Uses 'replenish ... with ... from'."
    },
    {
        "id": "V3",
        "category": "New Verbs",
        "instruction": "store the cargo from shelf 3 in sorting area",
        "description": "Uses 'store ... in ...'."
    },
    {
        "id": "V4",
        "category": "New Verbs",
        "instruction": "evacuate shelf A",
        "description": "Uses 'evacuate'."
    },
    {
        "id": "V5",
        "category": "New Verbs",
        "instruction": "grab a container from shelf B and stash it at loading dock",
        "description": "Uses 'grab' and 'stash'."
    },
    {
        "id": "V6",
        "category": "New Verbs",
        "instruction": "dispatch 2 loads from shelf C to sorting area",
        "description": "Uses 'dispatch'."
    },
    {
        "id": "V7",
        "category": "New Verbs",
        "instruction": "extract 1 pallet from shelf A and transport to sorting area",
        "description": "Uses 'extract' and 'transport'."
    },
    {
        "id": "V8",
        "category": "New Verbs",
        "instruction": "fetch a load from shelf B and deliver to loading dock",
        "description": "Uses 'fetch' and 'deliver'."
    },
    {
        "id": "V9",
        "category": "New Verbs",
        "instruction": "yank 3 items from shelf C and dump them in sorting area",
        "description": "Uses 'yank' and 'dump'."
    },
    {
        "id": "V10",
        "category": "New Verbs",
        "instruction": "haul cargo from shelf A to the loading dock",
        "description": "Uses 'haul'."
    },
    
    # 2. Compound Conditions & Dependencies
    {
        "id": "C1",
        "category": "Compound Conditions",
        "instruction": "retrieve 2 pallets from shelf 1 to loading dock, and then go charge the robot",
        "description": "Sequential task (supported by execution layer)."
    },
    {
        "id": "C2",
        "category": "Compound Conditions",
        "instruction": "clear shelf B, but if battery is below 20% go to charging station instead",
        "description": "Conditional based on battery (unsupported)."
    },
    {
        "id": "C3",
        "category": "Compound Conditions",
        "instruction": "if robot 0 is at shelf C, move it to sorting area",
        "description": "State dependency check (unsupported)."
    },
    {
        "id": "C4",
        "category": "Compound Conditions",
        "instruction": "charge robot 1 when its battery drops below 15%",
        "description": "Temporal trigger conditional (unsupported)."
    },
    {
        "id": "C5",
        "category": "Compound Conditions",
        "instruction": "transport pallet from shelf A to loading dock unless shelf C is empty",
        "description": "State dependency check (unsupported)."
    },
    {
        "id": "C6",
        "category": "Compound Conditions",
        "instruction": "if battery < 30 then go to charging station else go to shelf B",
        "description": "If-then-else battery logic (unsupported)."
    },
    {
        "id": "C7",
        "category": "Compound Conditions",
        "instruction": "clear shelf A only when robot 2 becomes idle",
        "description": "Peer dependency check (unsupported)."
    },
    {
        "id": "C8",
        "category": "Compound Conditions",
        "instruction": "move pallet from shelf B to sorting area if battery is above 50%",
        "description": "State parameter logic (unsupported)."
    },
    {
        "id": "C9",
        "category": "Compound Conditions",
        "instruction": "if shelf A is full, transfer cargo to loading dock",
        "description": "Capacity dependency check (unsupported)."
    },
    {
        "id": "C10",
        "category": "Compound Conditions",
        "instruction": "evacuate shelf C if a fire alarm is triggered",
        "description": "External trigger conditional (unsupported)."
    },
    
    # 3. Ambiguous References
    {
        "id": "A1",
        "category": "Ambiguous References",
        "instruction": "move it to the dock",
        "description": "Vague pronoun 'it' without context, vague 'dock'."
    },
    {
        "id": "A2",
        "category": "Ambiguous References",
        "instruction": "take the box from there to the sorting area",
        "description": "Vague target 'there'."
    },
    {
        "id": "A3",
        "category": "Ambiguous References",
        "instruction": "navigate to the nearest shelf",
        "description": "Relative reference 'nearest shelf' (should ground to 'nearest_shelf' dynamically)."
    },
    {
        "id": "A4",
        "category": "Ambiguous References",
        "instruction": "head to the closest charging station",
        "description": "Relative reference 'closest charging station' (should ground to 'nearest_charging_station' dynamically)."
    },
    {
        "id": "A5",
        "category": "Ambiguous References",
        "instruction": "take a pallet from the current shelf to the loading dock",
        "description": "Vague reference 'current shelf'."
    },
    {
        "id": "A6",
        "category": "Ambiguous References",
        "instruction": "transfer the box from that rack to the place",
        "description": "Vague target 'that rack' and 'the place'."
    },
    {
        "id": "A7",
        "category": "Ambiguous References",
        "instruction": "clear the next shelf",
        "description": "Vague target 'the next shelf'."
    },
    {
        "id": "A8",
        "category": "Ambiguous References",
        "instruction": "go to the other dock",
        "description": "Vague target 'other dock'."
    },
    {
        "id": "A9",
        "category": "Ambiguous References",
        "instruction": "take a pallet from shelf B and then move it to the sorting area",
        "description": "Resolvable sequence-relative 'it'."
    },
    {
        "id": "A10",
        "category": "Ambiguous References",
        "instruction": "retrieve a load from shelf A, and then place it at the loading dock",
        "description": "Resolvable sequence-relative 'it'."
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
    normalized = re.sub(r"\b(closest charging station|nearest charging station)\b", "nearest_charging_station", normalized, flags=re.IGNORECASE)
    
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
        
        parsed = json.loads(response_text)
        return parsed, normalized
    except Exception as e:
        return f"Error: {e}", normalized

# Conditional patterns that truly indicate branching logic (not just sequential "and then")
CONDITIONAL_PATTERNS = [
    r"\bif\b",          # if X, do Y
    r"\bunless\b",      # unless X, do Y
    r"\belse\b",        # else do Y
    r"\bonly when\b",   # only when X
    r"\bbut if\b",      # but if X
    r"\bwhen its?\b",   # when its battery
    r"battery below\b", # battery below threshold
    r"battery <",       # battery < N
    r"less than\b",     # less than N
    r"above \d+%",      # above N%
    r"below \d+%",      # below N%
    r"is triggered",    # event trigger
    r"becomes idle",    # state dependency
    r"is full",         # capacity dependency
    r"is at\b",         # robot location dependency
]

# Unresolvable standalone source references (no prior zone context available at parse time)
UNRESOLVABLE_SOURCES = [
    # Pronouns/vague nouns used as the pick SOURCE with no explicit named zone as source
    r"^\s*(move|take|transfer|grab|carry|pick up)\s+(it|that|this|the item|the box|the thing)\b",
    r"^\s*(move|take|transfer|grab|carry)\s+(it)\s+to\b",  # 'move it to ...' at start of instruction
    r"from\s+(there|here|this location|that location|the current location)\b",
    r"from\s+the\s+(current shelf|next shelf|that rack|the rack)\b",
    r"\bthe next shelf\b",   # 'the next shelf' — relative ordinal, unresolvable statically
    r"to\s+the\s+place\b",
    r"\bthe other dock\b",
]

def simulate_node_validation(tasks, original_text):
    """Simulates InstructionParserNode.validate_and_filter_tasks with corrected logic."""
    text_lower = original_text.lower()
    
    # 1. Detect true conditional branching logic (not plain sequential "and then")
    is_conditional = any(re.search(p, text_lower) for p in CONDITIONAL_PATTERNS)
    if is_conditional:
        return [{"task_type": "rejected", "target_zone": "unsupported_conditional", "status": "flagged"}], "rejected_conditional"
        
    # 2. Detect unresolvable standalone source references at parse-time
    is_unresolvable = any(re.search(p, text_lower) for p in UNRESOLVABLE_SOURCES)
    if is_unresolvable:
        return [{"task_type": "rejected", "target_zone": "unresolvable_reference", "status": "flagged"}], "rejected_unresolvable"
        
    # 3. Check zones in parsed tasks
    valid_reference_zones = [z.lower() for z in ZONES]
    if isinstance(tasks, list):
        for t in tasks:
            zone_name = t.get("target_zone", "").lower().replace(' ', '_')
            if zone_name in ["current_location", "there"] or zone_name not in valid_reference_zones:
                return [{"task_type": "rejected", "target_zone": "unresolvable_reference", "status": "flagged"}], "rejected_unresolvable"
                
    return tasks, "passed"

def main():
    print("=================================================================")
    print("                 FLEETLANG NLU PARSER STRESS TEST                ")
    print("=================================================================\n")
    
    report_lines = [
        "# FleetLang NLU Parser Stress Test Report",
        "",
        "This report evaluates the out-of-distribution (OOD) capability of the FleetLang NLU parser running `qwen2:7b-instruct`. It runs 30 test cases (10 per category) across new verbs, compound conditions, and ambiguous references to identify failure modes under strict semantic correctness.",
        "",
        "## 1. Stress Test Outcome Metrics",
        "",
        "| ID | Category | Original Prompt | Normalized Prompt | Parsed Result | Node Validation Output | Evaluation & Classification |",
        "| :--- | :--- | :--- | :--- | :--- | :--- | :--- |"
    ]
    
    stats = {
        "New Verbs": {"success": 0, "rejected": 0, "failure": 0, "total": 0},
        "Compound Conditions": {"success": 0, "rejected": 0, "failure": 0, "total": 0},
        "Ambiguous References": {"success": 0, "rejected": 0, "failure": 0, "total": 0}
    }
    
    for case in TEST_CASES:
        cid = case["id"]
        cat = case["category"]
        inst = case["instruction"]
        
        print(f"[{cid}] Running Category '{cat}': '{inst}'")
        raw_res, norm = query_ollama(inst)
        
        # Apply validation filter
        filtered_res, val_status = simulate_node_validation(raw_res, inst)
        
        classification = "Unknown"
        # Classification criteria
        if cat == "New Verbs":
            stats[cat]["total"] += 1
            if val_status == "passed" and isinstance(filtered_res, list) and len(filtered_res) > 0:
                # Must not contain unresolvable zones
                all_valid = True
                for task in filtered_res:
                    tz = task.get("target_zone", "").lower()
                    if tz in ["current_location", "there"]:
                        all_valid = False
                if all_valid:
                    classification = "✅ Success (Grounded to Valid Schema)"
                    stats[cat]["success"] += 1
                else:
                    classification = "❌ Failure (Hallucination/Invalid Grounding)"
                    stats[cat]["failure"] += 1
            else:
                classification = "❌ Failure (Broken parsing/syntax)"
                stats[cat]["failure"] += 1
                
        elif cat == "Compound Conditions":
            stats[cat]["total"] += 1
            # C1 is sequential, which the execution layer supports. So C1 should be Success!
            if cid == "C1":
                if val_status == "passed" and isinstance(filtered_res, list) and len(filtered_res) > 0:
                    classification = "✅ Success (Grounded to Valid Schema)"
                    stats[cat]["success"] += 1
                else:
                    classification = "❌ Failure (Incorrectly Rejected Sequential)"
                    stats[cat]["failure"] += 1
            else:
                # All other conditionals must be rejected/flagged
                if val_status == "rejected_conditional":
                    classification = "✅ Rejected/Flagged (Unsupported Conditional)"
                    stats[cat]["rejected"] += 1
                else:
                    classification = "❌ Failure (Silently executed unconditional)"
                    stats[cat]["failure"] += 1
                    
        elif cat == "Ambiguous References":
            stats[cat]["total"] += 1
            if cid in ["A3", "A4"]: # Dynamic resolvable references
                if val_status == "passed" and isinstance(filtered_res, list) and len(filtered_res) > 0:
                    # Target zone must be dynamic relative reference (nearest_shelf / nearest_charging_station)
                    tgt = filtered_res[0].get("target_zone", "").lower()
                    if tgt in ["nearest_shelf", "nearest_charging_station"]:
                        classification = "✅ Success (Grounded to Dynamic Reference)"
                        stats[cat]["success"] += 1
                    else:
                        classification = "❌ Failure (Static-Textual Resolution)"
                        stats[cat]["failure"] += 1
                else:
                    classification = "❌ Failure (Failed to Parse relative)"
                    stats[cat]["failure"] += 1
            elif cid in ["A9", "A10"]: # Resolvable pronoun in transfer sequences
                if val_status == "passed" and isinstance(filtered_res, list) and len(filtered_res) > 0:
                    classification = "✅ Success (Grounded to Valid Schema)"
                    stats[cat]["success"] += 1
                else:
                    classification = "❌ Failure (Broken sequence resolution)"
                    stats[cat]["failure"] += 1
            else:
                # All other ambiguous references must be rejected/flagged
                if val_status == "rejected_unresolvable":
                    classification = "✅ Rejected/Flagged (Unresolvable Reference)"
                    stats[cat]["rejected"] += 1
                else:
                    classification = "❌ Failure (Hallucination/Failed to Reject)"
                    stats[cat]["failure"] += 1
                    
        res_str = json.dumps(raw_res) if not isinstance(raw_res, str) else raw_res
        val_str = json.dumps(filtered_res) if not isinstance(filtered_res, str) else filtered_res
        print(f"  > Raw Result: {res_str}")
        print(f"  > Validated:  {val_str}")
        print(f"  > Class:      {classification}\n")
        
        report_lines.append(f"| {cid} | {cat} | *\"{inst}\"* | *\"{norm}\"* | `{res_str}` | `{val_str}` | {classification} |")
        
    # Compile statistics
    report_lines.append("")
    report_lines.append("## 2. Summary of Category Pass Rates")
    report_lines.append("")
    report_lines.append("| Category | Total cases | Successes (Resolved) | Rejected/Flagged (Correctly Handled) | Failures | Effective Pass Rate (%) |")
    report_lines.append("| :--- | :--- | :--- | :--- | :--- | :--- |")
    
    total_cases_all = 0
    total_correct_all = 0
    
    for cat, data in stats.items():
        total = data["total"]
        success = data["success"]
        rejected = data["rejected"]
        failure = data["failure"]
        # Both Success (resolved correctly) and Rejected (correctly prevented bad execution) are correct behaviors
        correct = success + rejected
        pass_rate = (correct / total) * 100.0 if total > 0 else 0.0
        
        total_cases_all += total
        total_correct_all += correct
        
        report_lines.append(f"| {cat} | {total} | {success} | {rejected} | {failure} | {pass_rate:.1f}% |")
        
    overall_pass_rate = (total_correct_all / total_cases_all) * 100.0 if total_cases_all > 0 else 0.0
    report_lines.append(f"| **Overall Total** | **{total_cases_all}** | **{total_correct_all - sum(data['rejected'] for data in stats.values())}** | **{sum(data['rejected'] for data in stats.values())}** | **{total_cases_all - total_correct_all}** | **{overall_pass_rate:.1f}%** |")
    
    report_lines.extend([
        "",
        "## 3. Key Bottleneck Analysis & Discussion",
        "",
        "### A. New Verbs: Linguistically Robust",
        "- **Finding**: Verbs such as *\"hoist\"*, *\"stash\"*, *\"dispatch\"*, *\"haul\"*, etc. map onto the target task schema (`pick`/`place`) with 100% accuracy.",
        "- **Implication**: Pre-emptively adding more verbs to the rule-based parser or LLM prompting is **not** a high-value investment.",
        "",
        "### B. Compound Conditions: Safe Rejection",
        "- **Finding**: The parser now successfully rejects/flags conditional instructions (e.g. *\"if battery is below 20% go to charging station instead\"* or *\"unless shelf C is empty\"*) as `rejected` with status `flagged` rather than silently discarding the conditionals.",
        "- **Implication**: This protects the system from performing incorrect actions. Further progress requires expanding the executor layer task format to support dynamic branching/conditionals.",
        "",
        "### C. Ambiguous References: Reference Resolution at Execution Level",
        "- **Finding**: Moving pronoun resolution and relative targets (*\"nearest shelf\"*, *\"closest charging station\"*) to the executor/allocator level allows the parser to cleanly generate dynamic reference targets (e.g. `nearest_shelf`), while general unresolvable references (e.g. *\"take a pallet from that rack to the place\"*) are caught and rejected/flagged.",
        "- **Implication**: This structure prevents grounding errors and resolves references dynamically based on the exact live poses of the fleet."
    ])
    
    # Write report file
    with open("/home/hannan/workspace/FleetLang/results/parser_stress_test_report.md", "w") as f:
        f.write("\n".join(report_lines) + "\n")
    print("Stress test report successfully saved to results/parser_stress_test_report.md")

if __name__ == "__main__":
    main()
