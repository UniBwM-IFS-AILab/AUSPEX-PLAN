from .llm_models import *

class SayModel():
    def __init__(self, use_llm="gpt"):
        '''
        LLM interface
        '''
        if use_llm == "gpt":
            self._llm = GPT_Interface()
        elif use_llm == "claude":
            self._llm = CLAUDE_Interface()
        elif use_llm == "gemini":
            self._llm = Gemini_Interface()
        # elif use_llm == "llama":
        #     self._llm = Llama_Interface()
        # elif use_llm == "llama2":
        #     self._llm = Llama2_Interface()
    

    def compute_llm_score(self, system_prompt, state, llm_use_state, actions, _synchronous_calls=False):
        print(f'Now thinking ... ')
        
        print("Getting best results...\n")
        if llm_use_state:
            state_str = f'The current state of the quadcopter is {state._airborne_state}'# and it is at waypoint {state._currentWaypoint}.'

            CLASSIFICATION_PROMPT = """You will be given a possible next skill for this drone control scenario.
            Classify whether this skill, is useful for completing the given higher goal, into one of the two categories: TRUE and FALSE.
            Thereby, take the previous actions and the final goal into account.
            Return only the name of the category, and nothing else.
            MAKE SURE your output is one of the two categories stated.""" + state_str + """
            The possible next skill to be classified: {next_skill}"""
        else:
            CLASSIFICATION_PROMPT = """You will be given a possible next skill for this drone control scenario.
            Classify whether this skill, is useful for completing the given higher goal, into one of the two categories: TRUE and FALSE.
            Thereby, take the previous actions and the final goal into account.
            Return only the name of the category, and nothing else.
            MAKE SURE your output is one of the two categories stated.
            The possible next skill to be classified: {next_skill}"""
        
        if _synchronous_calls:
            scores_percent = self._llm.request_llm_sync(system_prompt, CLASSIFICATION_PROMPT, actions)
        else:
            scores_percent = self._llm.request_llm_parallel(system_prompt, CLASSIFICATION_PROMPT, actions)
        print("Done.")
        return scores_percent

    def parseNL2Param(self, nl, detections, detection_color, locations, hl_skills):
        return self._llm.parseNL2Param(nl, detections, detection_color,locations, hl_skills)