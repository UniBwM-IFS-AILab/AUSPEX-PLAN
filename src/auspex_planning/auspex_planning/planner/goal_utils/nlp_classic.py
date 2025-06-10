import copy
from typing import List, Tuple, Optional
import json
from auspex_msgs.msg import Goal
import re
from nltk.tokenize import word_tokenize
from nltk.corpus import stopwords
import string


class NLP_CLASSIC:
    def __init__(self, kb_client=None):
        self._kb_client = kb_client

        self.goals = [
            'SEARCH',
            'FOLLOW',
            'FIND',
            'INSPECT',
            'SURVEY',
            'MONITOR',
            'LAND',
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
            'NOT_SPECIFIED'
        ]

    def preprocess(self, text):
        text = text.lower()
        text = re.sub(rf"[{string.punctuation}]", "", text)
        tokens = word_tokenize(text)
        tokens = [t for t in tokens if t not in stopwords.words("english")]
        return tokens

    def match_goal(self, user_input):
        tokens = self.preprocess(user_input)
        matched = {}
        for goal, keywords in self.goals.items():
            score = sum(1 for word in tokens if word in keywords)
            if score > 0:
                matched[goal] = score
        return max(matched, key=matched.get) if matched else None

    def parse_instruction2dict(self, instruction: str) -> dict:
        example_return = {"goals": [{"type": "SEARCH", "locations": ["openareas"],"additional_info": []},{"type": "FIND", "objects": ["person"],"colors": ["NOT_SPECIFIED"],"additional_info": []}], "constraints": [{"type": "SEARCH"},{"type": "FIND"}]}
        return example_return


        {{
            "goals": ["goal1", "goal2"],
            "objects": ["object1", "object2"],
            "colors": ["color1", "color2"],
            "locations": ["location1", "location2"],
            "additional_info": ["info1", "info2"]
        }}

        goals = self.match_goal(instruction)

