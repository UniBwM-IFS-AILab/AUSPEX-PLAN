
from auspex_planning.planner.utils.llm_planner_utils.gpt_models import MultiLLMClient
import copy
from typing import List, Tuple, Optional
import json
from auspex_msgs.msg import Goal

class NLP_LLM:
    def __init__(self, provider: str = "openai", model: str = "gpt-4o-mini", kb_client=None):
        self._kb_client = kb_client
        self.provider = provider.lower()
        self._llm = MultiLLMClient(provider, model)

    def build_json_prompt(self, goals, constraints, parameters) -> str:
        return f"""
                "You are a mission-planning assistant. Your job is to parse user instructions and return ONLY valid JSON. Parse the user instruction and output **ONLY** a JSON object with this exact schema:

                {{
                "goals": [
                    {{
                    "type": STRING from {goals},
                    "locations": ARRAY of STRINGs from {parameters} (only include for SEARCH goals),
                    "last_known_position": BOOLEAN,
                    "objects": ARRAY of STRINGs from {parameters} (only include for FIND goals),
                    "colors": ARRAY of STRINGs from {parameters} (aligns with objects and only include for FIND goals; default to "NOT_SPECIFIED" if unclear),
                    "additional_info": [{{"cam_type": STRING from ["eo","ir","NOT_SPECIFIED"]}}]
                    }}
                ],
                "constraints": [
                    {{ "type": STRING from {constraints} }},
                    ...
                ]
                }}

                - Only include goals mentioned in the text.
                - For SEARCH: include any location params mentioned.
                - last_known_position = true if the phrase "last known position" appears; otherwise false.
                - For FIND: extract mentioned objects and match colors; default missing colors to "NOT_SPECIFIED".
                - Always include constraint WITHIN_BOUNDS; include other constraints only if clearly mentioned.
                - Default cam_type to "eo" unless "ir" explicitly appears.
                - **No additional keys** and **no commentary—just** valid **JSON**.

                Example:
                User: "Search open areas and urban areas for a person with a blue backpack using EO camera. "
                Output:
                {{
                "goals": [
                    {{
                    "type": "SEARCH",
                    "locations": ["openareas", "urbanareas"],
                    "last_known_position": false,
                    "additional_info": [{{"cam_type": "eo"}}]
                    }},
                    {{
                    "type": "FIND",
                    "objects": ["person", "backpack"],
                    "colors": ["NOT_SPECIFIED", "blue"],
                    "additional_info": [{{"cam_type": "eo"}}]
                    }}
                ],
                "constraints": [{{"type": "WITHIN_BOUNDS"}}]
                }}

                Here is the input:
                """

    def parse_instruction2dict(self, instruction: str) -> dict:
        """
        Parses natural language instruction into goal/constraint components.
        If no goals are parsed, it requests confirmation from the user via a symbolic function.

        Returns:
            dict: Parsed variables as a dictionary.
        """
        #example_return = {'goals': [{'type': 'SEARCH', 'locations': ['openareas'], 'last_known_position': False, 'additional_info': [{'cam_type': 'eo'}]}, {'type': 'FIND', 'objects': ['person'], 'colors': ['NOT_SPECIFIED'], 'additional_info': [{'cam_type': 'eo'}]}], 'constraints': [{'type': 'WITHIN_BOUNDS'}, {'type': 'SPEED_LIMIT'}]}
        #return example_return

        goals = [
            'SEARCH',
            'FIND',
            'NOT_SPECIFIED'
        ]

        constraints = [
            'MISSION_TIME_LIMIT',      # Adhere to mission time constraints
            'WITHIN_BOUNDS',           # Stay within specified geographic bounds
            'MAX_ALTITUDE',            # Do not exceed maximum altitude
            'MIN_ALTITUDE',            # Do not go below minimum altitude
            'NO_FLY_ZONES',            # Avoid specified no-fly zones
            'BATTERY_LIMIT',           # Operate within battery constraints
            'WEATHER_CONSTRAINTS',     # Consider weather limitations
            'SPEED_LIMIT',             # Do not exceed speed limits
            'COMMUNICATION_RANGE',     # Stay within communication range
            'NOT_SPECIFIED'
        ]

        # Add last known position to parameters
        parameters = [
            'person',
            'vehicle',
            'uav',
            'red',
            'green',
            'blue',
            'yellow',
            'orange',
            'purple',
            'black',
            'white',
            'gray',
            'openareas'
            'urbanareas',
            'waters',
            'woods',
            'pois',
            'NOT_SPECIFIED'
        ]

        prompt = self.build_json_prompt(goals=goals, constraints=constraints, parameters=parameters) + instruction

        while True:
            # Generate the prompt for the LLM
            prompt = copy.deepcopy(prompt) + instruction
            result = self._llm.query(prompt=prompt, max_tokens=16384)

            if "error" in result:
                raise RuntimeError(f"LLM Error: {result['error']}")



            try:
                if self.provider == "deepseek":
                    raw_response = result["response"].strip()
                    if raw_response.lstrip().startswith("```json"):
                        raw_response = "\n".join(raw_response.lstrip().splitlines()[1:])
                        if raw_response.strip().endswith("```"):
                            raw_response = "\n".join(raw_response.strip().splitlines()[:-1])
                    response_json = json.loads(raw_response)
                    goals = response_json.get("goals", ["NOT_SPECIFIED"])
                else:
                    response_json = json.loads(result["response"].strip())
                    goals = response_json.get("goals", ["NOT_SPECIFIED"])

                # if "NOT_SPECIFIED" in goals or not goals:
                #     instruction = self.confirm_by_user(instruction)
                # else:
                return response_json

            except json.JSONDecodeError:
                print("Failed to parse LLM response as JSON. Raw response:")
                print(result["response"].strip())
                return {}

    def confirm_by_user(self, instruction: str) -> str:
        print(f"Goal could not be determined for the instruction: '{instruction}'")
        print("Please refine the instruction or provide additional details.")
        new_input = raw_input("Refined instruction: ")
        if not new_input:
            print("No input provided. Using the original instruction.")
            return instruction
        refined_instruction = instruction + new_input
        return refined_instruction