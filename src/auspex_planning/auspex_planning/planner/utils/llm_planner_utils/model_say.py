import copy

import concurrent
import numpy as np
from .gpt_models import MultiLLMClient

class SayModel():
    def __init__(self, provider: str = "openai", model: str = "gpt-4o-mini"):
        '''
        LLM interface
        '''
        self._llm = MultiLLMClient(provider, model)

    def compute_llm_score(self, system_prompt:str, actions:list, state = None, synchronous_call=False):
        print(f'Now thinking ... ')

        print("Getting best results...\n")
        if state is not None:
            state_str = f'The current state of the quadcopter is {state}'

            user_prompt = """You will be given a possible next action for this drone control scenario.
            Classify whether this action, is useful for completing the given higher goal, into one of the two categories: TRUE and FALSE.
            Thereby, take the previous actions and the final goal into account.
            Return only the name of the category, and nothing else.
            MAKE SURE your output is one of the two categories stated.""" + state_str + """
            The possible next action to be classified: {next_action}"""
        else:
            user_prompt = """You will be given a possible next action for this drone control scenario.
            Classify whether this action, is useful for completing the given higher goal, into one of the two categories: TRUE and FALSE.
            Thereby, take the previous actions and the final goal into account.
            Return only the name of the category, and nothing else.
            MAKE SURE your output is one of the two categories stated.
            The possible next action to be classified: {next_action}"""

        if synchronous_call:
            scores_percent = self.query_sync(system_prompt, user_prompt, actions)
        else:
            scores_percent = self.query_async(system_prompt, user_prompt, actions)
        return scores_percent


    def query_sync(self, system_prompt:str, user_prompt:str, actions:list):
        """
        Query the LLM synchronously.
        """
        messages = [{"role": "system", "content": system_prompt}]
        scores = []
        for action in actions:
            mod_messages = copy.deepcopy(messages)
            mod_messages.append({"role": "user", "content": user_prompt.format(next_action=action)})

            response = self._llm.query(prompt=mod_messages, max_tokens=50)
            prob = response["logprobs"][0].logprob
            if prob is None:
                prob = 0.0
                if 'true' in response["response"].lower():
                    prob = 1.0
            else:
                prob = np.exp(prob)
                if 'false' in response["response"].lower():
                    prob = 1 - prob

            scores.append(prob)

        scores_percent = np.array(scores) / np.sum(scores)
        return scores_percent


    def query_async(self, system_prompt:str, user_prompt:str, actions:list):
        """
        Query the LLM asynchronously.
        """
        messages = [{"role": "system", "content": system_prompt}]
        scores = [None] * len(actions)  # Placeholder for results to maintain order
        futures = []
        batch_size = 150

        with concurrent.futures.ThreadPoolExecutor() as executor:
            for batch_start in range(0, len(actions), batch_size):
                batch = actions[batch_start:min(batch_start + batch_size, len(actions))]
                for i, action in enumerate(batch, start=batch_start):
                    mod_messages = copy.deepcopy(messages)
                    mod_messages.append({"role": "user", "content": user_prompt.format(next_action=action)})
                    futures.append(executor.submit(self.process_message, i, mod_messages))

                for future in concurrent.futures.as_completed(futures):
                    index, prob = future.result()
                    scores[index] = prob
        scores_percent = np.array(scores) / np.sum(scores)
        return scores_percent

    def process_message(self, index, message):
        """
        Process a message and return the index and probability.
        """
        response = self._llm.query(prompt=message, max_tokens=50)
        prob = response["logprobs"][0].logprob
        if prob is None:
            prob = 0.0
            if 'true' in response["response"].lower():
                prob = 1.0
        else:
            prob = np.exp(prob)
            if 'false' in response["response"].lower():
                prob = 1 - prob
        return index, prob