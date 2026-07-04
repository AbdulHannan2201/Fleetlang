#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from msgs.msg import TaskList, Task, SemanticMap
import re
import json
import urllib.request
import urllib.error

class InstructionParserNode(Node):
    def __init__(self):
        super().__init__('instruction_parser_node')
        
        self.declare_parameter('llm_url', 'http://localhost:11434/api/generate')
        self.declare_parameter('llm_model', 'qwen2:7b-instruct')
        
        self.llm_url = self.get_parameter('llm_url').value
        self.llm_model = self.get_parameter('llm_model').value
        
        self.semantic_map = None
        self.task_counter = 0
        
        # Subscriber for semantic map (Transient Local QoS to match the publisher)
        qos_profile = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.map_sub = self.create_subscription(
            SemanticMap,
            '/fleet/semantic_map',
            self.map_callback,
            qos_profile
        )
        
        # Instruction Subscriber
        self.instruction_sub = self.create_subscription(
            String,
            '/fleet/instruction',
            self.instruction_callback,
            10
        )
        
        # Task List Publisher
        self.task_list_pub = self.create_publisher(
            TaskList,
            '/fleet/task_list',
            10
        )
        
        self.get_logger().info('Instruction Parser Node (LLM-enabled) initialized. Waiting for semantic map.')

    def get_zone_target_pose(self, zone):
        import copy
        pose = copy.deepcopy(zone.center)
        if zone.zone_type == "shelf":
            # Offset target Y to the aisle in front of the shelf (Y = 1.0)
            pose.position.y = 1.0
        return pose

    def map_callback(self, msg):
        self.semantic_map = msg
        self.get_logger().info(f'Received semantic map with {len(msg.zones)} zones.')

    def instruction_callback(self, msg):
        instruction_text = msg.data
        self.get_logger().info(f"Parsing instruction: '{instruction_text}'")
        
        if self.semantic_map is None:
            self.get_logger().error("Cannot parse instruction: Semantic map not yet received!")
            return
            
        task_list_msg = self.parse_instruction(instruction_text)
        self.task_list_pub.publish(task_list_msg)
        self.get_logger().info(f"Published task list with {len(task_list_msg.tasks)} tasks.")

    def parse_instruction(self, text, use_llm=True):
        import re
        # Standardize shelf numbers and correct typos (e.g. shelp/shlef/shefl/shelves/shelfs -> shelf)
        text = re.sub(r"\b(shelp|shlef|shefl|shelves|shelfs)\b", "shelf", text, flags=re.IGNORECASE)
        text = re.sub(r"\bshelf\s*1\b", "shelf_A", text, flags=re.IGNORECASE)
        text = re.sub(r"\bshelf\s*2\b", "shelf_B", text, flags=re.IGNORECASE)
        text = re.sub(r"\bshelf\s*3\b", "shelf_C", text, flags=re.IGNORECASE)

        task_list_msg = TaskList()
        task_list_msg.instruction = text
        
        if not self.semantic_map:
            # Standalone mode helper or error
            return task_list_msg

        zone_names = [zone.name for zone in self.semantic_map.zones]
        
        tasks = []
        parsed_successfully = False
        
        if use_llm:
            try:
                tasks = self.query_llm_parser(text, zone_names)
                if tasks:
                    parsed_successfully = True
                    self.get_logger().info(f"LLM successfully parsed {len(tasks)} tasks.")
            except Exception as e:
                self.get_logger().warn(f"LLM parsing failed: {e}. Falling back to rule-based parser.")
        
        if not parsed_successfully:
            tasks = self.parse_instruction_fallback(text)
            self.get_logger().info(f"Fallback parser generated {len(tasks)} tasks.")
            
        tasks = self.post_process_tasks(tasks, text)
        task_list_msg.tasks = tasks
        return task_list_msg

    def post_process_tasks(self, base_tasks, text):
        import re
        import copy
        
        # 1. Parse quantity (default to 1)
        quantity = 1
        qty_match = re.search(r"(\d+)\s*(?:pallet|item|box|pack|unit|load)s?", text.lower())
        if qty_match:
            quantity = int(qty_match.group(1))
        else:
            qty_match_standalone = re.search(r"(?:offload|pickup|move|transfer)\s+(\d+)", text.lower())
            if qty_match_standalone:
                quantity = int(qty_match_standalone.group(1))
                
        # 2. Parse robot count
        robots_count = None
        bot_match = re.search(r"(?:using|with|via)\s*(\d+)\s*(?:robot|bot)s?", text.lower())
        if bot_match:
            robots_count = int(bot_match.group(1))
            
        self.get_logger().info(f"Post-processing tasks: quantity={quantity}, robots_count={robots_count}")
        
        if not base_tasks:
            return base_tasks
            
        # If it's a charge or robot-specific task already, don't duplicate/override
        if len(base_tasks) == 1 and ("charge" in base_tasks[0].task_type or "robot" in base_tasks[0].task_id):
            return base_tasks
            
        processed_tasks = []
        task_idx = 0
        
        for q in range(quantity):
            # If robots_count is specified, we assign this sequence to robot_(q % robots_count)
            robot_suffix = f"_robot_{q % robots_count}" if robots_count is not None else ""
            
            for base_t in base_tasks:
                task_idx += 1
                t = copy.deepcopy(base_t)
                t.task_id = f"task_{task_idx}_{t.task_type}{robot_suffix}"
                processed_tasks.append(t)
                
        return processed_tasks

    def query_llm_parser(self, text, zone_names):
        prompt = (
            "You are a warehouse task planner. Translate the following natural language instruction into a sequence of structured tasks.\n"
            f"Valid zone names: {zone_names}\n"
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
            "Instruction: \"clear shelf_B rack\"\n"
            "Output: [{\"task_type\": \"pick\", \"target_zone\": \"shelf_B\", \"priority\": 2}, {\"task_type\": \"place\", \"target_zone\": \"sorting_area\", \"priority\": 2}]\n\n"
            "Instruction: \"charge robot_0 at charging_station\"\n"
            "Output: [{\"task_type\": \"charge\", \"target_zone\": \"charging_station\", \"priority\": 3}]\n\n"
            "Instruction: \"go to sorting_area\"\n"
            "Output: [{\"task_type\": \"go_to\", \"target_zone\": \"sorting_area\", \"priority\": 0}]\n\n"
            f"Instruction: \"{text}\"\n"
            "Output ONLY raw JSON array, no extra words or markdown:"
        )
        
        # Sync parameters
        self.llm_url = self.get_parameter('llm_url').value
        self.llm_model = self.get_parameter('llm_model').value
        
        # REST API request to local Ollama service
        url = self.llm_url
        data = {
            "model": self.llm_model,
            "prompt": prompt,
            "stream": False,
            "options": {
                "temperature": 0.0  # Greedy decoding for deterministic task generation
            }
        }
        
        req = urllib.request.Request(
            url,
            data=json.dumps(data).encode('utf-8'),
            headers={'Content-Type': 'application/json'}
        )
        
        with urllib.request.urlopen(req, timeout=8.0) as response:
            res_body = json.loads(response.read().decode('utf-8'))
            response_text = res_body.get('response', '').strip()
            
        # Clean response text in case LLM outputs markdown block
        if response_text.startswith("```json"):
            response_text = response_text[7:]
        if response_text.endswith("```"):
            response_text = response_text[:-3]
        response_text = response_text.strip()
        
        parsed_json = json.loads(response_text)
        if not isinstance(parsed_json, list):
            raise ValueError("LLM did not return a JSON array.")
            
        tasks = []
        for item in parsed_json:
            t_type = item.get("task_type")
            t_zone = item.get("target_zone")
            priority = item.get("priority", 1)
            
            # Grounding: Verify zone name is known
            zone = self.find_zone_by_name(t_zone)
            if not zone:
                self.get_logger().warn(f"LLM hallucinated/unrecognized zone '{t_zone}'. Skipping task.")
                continue
                
            self.task_counter += 1
            t = Task()
            
            # Check for robot specific restrictions in charge commands
            robot_suffix = ""
            if t_type == "charge":
                match = re.search(r"robot_?(\d+)", text.lower())
                if match:
                    robot_suffix = f"robot_{match.group(1)}"
                    
            t.task_id = f"charge_{robot_suffix}_{self.task_counter}" if robot_suffix else f"task_{self.task_counter}_{t_type}"
            t.task_type = t_type
            t.target_zone = zone.name
            t.target_pose = self.get_zone_target_pose(zone)
            t.priority = priority
            t.status = "pending"
            tasks.append(t)
            
        return tasks

    def parse_instruction_fallback(self, text):
        text_lower = text.lower()
        # Match zones sorted by their position of occurrence in the text
        zone_positions = {}
        for zone in self.semantic_map.zones:
            normalized_name = zone.name.lower()
            spaced_name = normalized_name.replace('_', ' ')
            pos = text_lower.find(normalized_name)
            if pos == -1:
                pos = text_lower.find(spaced_name)
            if pos != -1:
                zone_positions[zone] = pos
        matched_zones = sorted(zone_positions.keys(), key=lambda z: zone_positions[z])
                
        tasks = []
        is_transfer = any(k in text_lower for k in ["move", "transfer", "transport", "deliver", "carry", "bring", "take", "offload", "retrieve", "unload"])
        is_clear = any(k in text_lower for k in ["clear", "empty", "cleanup"])
        is_charge = "charge" in text_lower
        is_goto = any(k in text_lower for k in ["go to", "navigate to", "visit", "go_to", "head to"])
        
        if is_transfer:
            source_zone = None
            target_zone = None
            
            for zone in matched_zones:
                normalized_name = zone.name.lower()
                spaced_name = normalized_name.replace('_', ' ')
                from_pattern = rf"from\s+(?:the\s+)?(?:{re.escape(normalized_name)}|{re.escape(spaced_name)})\b"
                to_pattern = rf"to\s+(?:the\s+)?(?:{re.escape(normalized_name)}|{re.escape(spaced_name)})\b"
                
                if re.search(from_pattern, text_lower):
                    source_zone = zone
                elif re.search(to_pattern, text_lower):
                    target_zone = zone
            
            if len(matched_zones) >= 2:
                if source_zone is None:
                    source_zone = matched_zones[0]
                if target_zone is None:
                    target_zone = matched_zones[1]
            elif len(matched_zones) == 1:
                source_zone = matched_zones[0]
                target_zone = self.find_zone_by_name("sorting_area")
            else:
                source_zone = self.find_zone_by_name("shelf_A")
                target_zone = self.find_zone_by_name("loading_dock")
                
            if source_zone and target_zone:
                self.task_counter += 1
                t1 = Task()
                t1.task_id = f"task_{self.task_counter}_pick"
                t1.task_type = "pick"
                t1.target_zone = source_zone.name
                t1.target_pose = self.get_zone_target_pose(source_zone)
                t1.priority = 1
                t1.status = "pending"
                tasks.append(t1)
                
                self.task_counter += 1
                t2 = Task()
                t2.task_id = f"task_{self.task_counter}_place"
                t2.task_type = "place"
                t2.target_zone = target_zone.name
                t2.target_pose = self.get_zone_target_pose(target_zone)
                t2.priority = 1
                t2.status = "pending"
                tasks.append(t2)
                
        elif is_clear:
            target_zone = matched_zones[0] if len(matched_zones) > 0 else self.find_zone_by_name("shelf_A")
            sorting_area = self.find_zone_by_name("sorting_area")
            
            if target_zone and sorting_area:
                self.task_counter += 1
                t1 = Task()
                t1.task_id = f"task_{self.task_counter}_pick"
                t1.task_type = "pick"
                t1.target_zone = target_zone.name
                t1.target_pose = self.get_zone_target_pose(target_zone)
                t1.priority = 2
                t1.status = "pending"
                tasks.append(t1)
                
                self.task_counter += 1
                t2 = Task()
                t2.task_id = f"task_{self.task_counter}_place"
                t2.task_type = "place"
                t2.target_zone = sorting_area.name
                t2.target_pose = self.get_zone_target_pose(sorting_area)
                t2.priority = 2
                t2.status = "pending"
                tasks.append(t2)
                
        elif is_charge:
            match = re.search(r"robot_?(\d+)", text_lower)
            robot_suffix = f"robot_{match.group(1)}" if match else ""
            charging_station = self.find_zone_by_name("charging_station")
            if charging_station:
                self.task_counter += 1
                t = Task()
                t.task_id = f"charge_{robot_suffix}_{self.task_counter}" if robot_suffix else f"task_{self.task_counter}_charge"
                t.task_type = "charge"
                t.target_zone = charging_station.name
                t.target_pose = self.get_zone_target_pose(charging_station)
                t.priority = 3
                t.status = "pending"
                tasks.append(t)
                
        elif is_goto or len(matched_zones) > 0:
            target_zone = matched_zones[0] if len(matched_zones) > 0 else self.find_zone_by_name("shelf_A")
            if target_zone:
                self.task_counter += 1
                t = Task()
                t.task_id = f"task_{self.task_counter}_goto"
                t.task_type = "go_to"
                t.target_zone = target_zone.name
                t.target_pose = self.get_zone_target_pose(target_zone)
                t.priority = 0
                t.status = "pending"
                tasks.append(t)
                
        return tasks

    def find_zone_by_name(self, name):
        if self.semantic_map is None:
            return None
        # Handle variations in naming/case from LLM
        clean_name = str(name).strip().lower().replace(' ', '_')
        for zone in self.semantic_map.zones:
            if zone.name.lower() == clean_name or zone.name.lower().replace('_', '') == clean_name.replace('_', ''):
                return zone
        return None

def main(args=None):
    rclpy.init(args=args)
    node = InstructionParserNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
