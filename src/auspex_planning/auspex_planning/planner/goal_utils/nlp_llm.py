import copy
from typing import List, Tuple, Optional
from auspex_planning.planner.llm_planner_utils.gpt_models import MultiLLMClient
import json
from auspex_msgs.msg import Goal

class NLP_LLM:
    def __init__(self, provider: str = "openai", model: str = "gpt-4o-mini", kb_client=None):
        self._kb_client = kb_client
        self._llm = MultiLLMClient(provider, model)

        self.goals = [
            'SEARCH',
            'FIND',
            'NOT_SPECIFIED'
        ]

        self.objects = [
            'person',
            'vehicle',
            'uav',
            'NOT_SPECIFIED'
        ]

        self.colors = [
            'red',
            'green',
            'blue',
            'yellow',
            'orange',
            'purple',
            'black',
            'white',
            'gray',
            'NOT_SPECIFIED'
        ]

        self.locations = [
            'openareas'
            'urbanareas',
            'waters',
            'pois'
            'NOT_SPECIFIED'
        ]

    def parse_instruction2dict(self, instruction: str) -> dict:
        """
        Parses natural language instruction into goal/constraint components.
        If no goals are parsed, it requests confirmation from the user via a symbolic function.

        Returns:
            dict: Parsed variables as a dictionary.
        """
        goals_str = ", ".join(self.goals)
        objects_str = ", ".join(self.objects)
        colors_str = ", ".join(self.colors)
        locations_str = ", ".join(self.locations)

        # example_return = {"goals": [{"type": "SEARCH", "locations": ["openareas"], "last_known_position": "true", "pois": [{"latitude": "48", "longitude":"11"}], "additional_info": [{"cam_type": "eo"}]},{"type": "FIND", "objects": ["person"],"colors": ["NOT_SPECIFIED"],"additional_info": []}], "constraints": [{"type": "SEARCH"},{"type": "FIND"}]}
        example_return = {"goals": [{"type": "SEARCH", "locations": ["openareas"], "last_known_position": "false", "additional_info": [{"cam_type": "eo"}]},{"type": "FIND", "objects": ["person"],"colors": ["NOT_SPECIFIED"],"additional_info": []}], "constraints": [{"type": "SEARCH"},{"type": "FIND"}]}
        return example_return

        #TODO Adapt NLP to this maybe also team_id (UL and drone team also )

        while True:
            # Generate the prompt for the LLM
            prompt = f"""
                Available goals: {goals_str}
                Available objects: {objects_str}
                Available colors: {colors_str}
                Available locations: {locations_str}

                Instruction: "{instruction}"

                Return a JSON object with:
                - goals (a list of one or more of the goals or ['NOT_SPECIFIED'])
                - objects (a list of one or more of the objects or ['NOT_SPECIFIED'])
                - colors (a list of one or more of the colors or ['NOT_SPECIFIED'])
                - locations (a list of one or more of the locations or ['NOT_SPECIFIED'])
                - additional_info (list of bullet points)
                in the following format:
                {{
                    "goals": ["goal1", "goal2"],
                    "objects": ["object1", "object2"],
                    "colors": ["color1", "color2"],
                    "locations": ["location1", "location2"],
                    "pois": [{"latitude": "48", "longitude":"11"},]
                    "additional_info": ["info1", "info2"]
                }}
                If no goals are given, return 'NOT_SPECIFIED' for the goals.
            """
            # Query the LLM
            result = self._llm.query(prompt=prompt, max_tokens=1000)
            if "error" in result:
                raise RuntimeError(f"LLM Error: {result['error']}")
            print(result["response"])
            try:
                response_text = result["response"]
                response_json = json.loads(response_text.strip())
                goals = response_json.get("goals", ["NOT_SPECIFIED"])

                if "NOT_SPECIFIED" in goals or not goals:
                    instruction = self.confirm_by_user(instruction)
                else:
                    return response_json

            except json.JSONDecodeError:
                print("Failed to parse LLM response as JSON. Raw response:")
                print(response_text)
                return {}

    def confirm_by_user(self, instruction: str) -> str:
        print(f"Goal could not be determined for the instruction: '{instruction}'")
        print("Please refine the instruction or provide additional details.")
        new_input = ""
        refined_instruction = instruction + new_input
        return refined_instruction