import os
import copy
import anthropic
import numpy as np
import concurrent.futures
from anthropic import HUMAN_PROMPT, AI_PROMPT

from transformers import BertTokenizer, BertModel, LlamaForCausalLM, LlamaTokenizer, GenerationConfig, T5Tokenizer, T5ForConditionalGeneration
import google.generativeai as genai
from openai import OpenAI, RateLimitError, APIError, Timeout, OpenAIError

#from llama import Llama

class GPT_Interface():
    def __init__(self):
        self._llm = None
        self._client = None

    def request_llm_sync(self, system_prompt, task, all_actions_list):
        if self._client == None:
            self._client = OpenAI()
        messages = [{"role": "system", "content": system_prompt}]
        scores = []

        for action in all_actions_list:
            mod_messages = messages
            mod_messages.append({"role": "user", "content": task.format(next_skill=action)})

            _, prob = self.process_message(0, mod_messages)

            scores.append(prob)

        scores = np.array(scores)
        scores_percent = []
        for score in scores:
            scores_percent.append(score/np.sum(scores))

        return np.array(scores_percent)


    def request_llm_parallel(self, system_prompt, task, all_actions_list):
        if self._client == None:
            self._client = OpenAI()
        messages = [{"role": "system", "content": system_prompt}]

        scores = [None] * len(all_actions_list)  # Placeholder for results to maintain order
        futures = []

        batch_size = 150
        with concurrent.futures.ThreadPoolExecutor() as executor:
            # Create a future for each message, submitting the process_message function
            for batch_start in range(0, len(all_actions_list), batch_size):
                batch = all_actions_list[batch_start:min(batch_start + batch_size, len(all_actions_list))]
                for i, action in enumerate(batch, start=batch_start):
                    mod_messages = messages.copy()
                    mod_messages.append({"role": "user", "content": task.format(next_skill=action)})
                    futures.append(executor.submit(self.process_message, i, mod_messages))

                # As each future completes, store the result in the correct order
                for future in concurrent.futures.as_completed(futures):
                    index, prob = future.result()
                    scores[index] = prob


        scores_percent = []
        scores_percent = scores / np.sum(scores)

        return np.array(scores_percent)

    def process_message(self, index, message):
        model = "gpt-4o-mini"
        #model = "gpt-4o-2024-08-06"

        API_RESPONSE = self.get_completion(
            message,
            model=model,
            logprobs=True,
            top_logprobs=1
        )
        top_logprobs = API_RESPONSE.choices[0].logprobs.content[0].top_logprobs

        prob = np.exp(top_logprobs[0].logprob)

        if top_logprobs[0].token.lower() == "false":
            prob = 1.0 - prob

        return index, prob


    def get_completion(
        self,
        messages: list[dict[str, str]],
        model: str = "gpt-4o-mini",
        max_tokens=500,
        temperature=0,
        stop=None,
        seed=123,
        tools=None,
        logprobs=None,
        top_logprobs=None,
    ) -> str:
        params = {
            "model": model,
            "messages": messages,
            "max_tokens": max_tokens,
            "temperature": temperature,
            "stop": stop,
            "seed": seed,
            "logprobs": logprobs,
            "top_logprobs": top_logprobs,
        }
        if tools:
            params["tools"] = tools
        retries = 5

        completion = self._client.chat.completions.create(**params)
        return completion