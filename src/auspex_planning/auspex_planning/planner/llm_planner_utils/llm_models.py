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


class Gemini_Interface():
    def __init__(self):
        genai.configure(api_key=os.environ['GEMINI_API_KEY'])

    def request_llm(self, system_prompt, task, all_actions_list):
        self._llm = genai.GenerativeModel("gemini-1.5-flash", system_instruction=system_prompt)
        '''
        Request LLm for every action
        '''
        scores = []

        for action in all_actions_list:

            completion = self._llm.generate_content(
                task,
                generation_config=genai.types.GenerationConfig(
                    max_output_tokens=20,
                    temperature=0.0,
                ),
            )

            print(completion)
            print(completion.text)
            
            if completion.content[0].text.lower() == "false":
                prob = 0.0
            elif completion.content[0].text.lower() == "true":
                prob = 1.0
            else:
                prob = 0.0

            scores.append(prob)
        scores = np.array(scores)
        scores_percent = []
        for score in scores:
            if np.sum(scores) == 0.0:
                scores_percent.append(0.0)
            else: 
                scores_percent.append(score/np.sum(scores))

        return scores_percent

class CLAUDE_Interface():
    def __init__(self):
        self._llm = anthropic.Anthropic(api_key=os.environ["ANTHROPIC_API_KEY"])

    def request_llm(self, system_prompt, task, all_actions_list):
        '''
        Request LLm for every action
        '''
        scores = []

        for action in all_actions_list:
            completion = self._llm.messages.create(
                model="claude-3-opus-20240229",
                max_tokens=50,
                system=system_prompt,
                messages=[{"role": "user", "content": task.format(next_skill=action)}],
                temperature=0
            )

            if completion.content[0].text.lower() == "false":
                prob = 0.0
            elif completion.content[0].text.lower() == "true":
                prob = 1.0
            else:
                prob = 0.0

            scores.append(prob)
        scores = np.array(scores)
        scores_percent = []
        for score in scores:
            if np.sum(scores) == 0.0:
                scores_percent.append(0.0)
            else: 
                scores_percent.append(score/np.sum(scores))

        return scores_percent
        

# class Llama2_Interface():
#     def __init__(self):   
#         self.lm_model = LlamaForCausalLM.from_pretrained("eachadea/vicuna-13b-1.1",
#                                                              device_map="auto",
#                                                              load_in_8bit=True)


# class Llama_Interface():
#     PATH_TOKENIZER = "/home/companion/python_ws/llama3/Meta-Llama-3.1-8B-Instruct/tokenizer.model"
#     PATH_MODEL_DIR= "/home/companion/python_ws/llama3/Meta-Llama-3.1-8B-Instruct/"

#     def __init__(self):   
#         os.environ["RANK"] = "0"
#         os.environ["WORLD_SIZE"] = "1"
#         os.environ["MASTER_ADDR"] = "localhost"
#         os.environ["MASTER_PORT"] = "29500"

#         self._llm = Llama.build(
#             ckpt_dir=self.PATH_MODEL_DIR,
#             tokenizer_path=self.PATH_TOKENIZER,
#             max_seq_len=2048,
#             max_batch_size=1,
#         )


#     def request_llm(self, system_prompt, task, all_actions_list):
#         scores = []

#         for action in all_actions_list:
#             n_tokens = len(self.get_tokenizer().encode(action, bos=False, eos=False))

#             results = self._llm.text_completion(
#                 [system_prompt + task.format(next_skill=action)],
#                 max_gen_len=10,
#                 temperature=0,
#                 echo=False,   
#                 logprobs=True
#             )

#             probs =  results[0]['logprobs']

#             probs = np.array(probs[-1:])
#             prob = np.exp(np.sum(probs)/float(n_tokens))
#             if "false" in results[0]['generation'].lower() and "true" not in results[0]['generation'].lower():
#                 prob = 1.0 - prob

#             scores.append(prob)

#         scores = np.array(scores)
#         scores_percent = []
#         for score in scores:
#             scores_percent.append(score/np.sum(scores))

#         return scores_percent

#     def get_tokenizer(self):
#         return self._llm.tokenizer



class GPT_Interface():
    def __init__(self):
        self._llm = None
        self._client = OpenAI()
        
    def request_llm_sync(self, system_prompt, task, all_actions_list):
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
        for score in scores:
            scores_percent.append(score/np.sum(scores))

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
        #rint(top_logprobs[0].token)
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

    def parseNL2Param(self, nl, detections, detection_colors, locations, hl_skills):
        locations_copy = copy.deepcopy(locations)
        locations_copy.append('NOT_SPECIFIED')

        detections_copy = copy.deepcopy(detections)
        detections_copy.append('NOT_SPECIFIED')

        detection_colors_copy = copy.deepcopy(detection_colors)
        detection_colors_copy.append('NOT_SPECIFIED')
        
        string_hl_skills = ", ".join(hl_skills)
        string_detections = ", ".join(detections_copy)
        string_detection_colors = ", ".join(detection_colors_copy)
        string_locations = ", ".join(locations_copy)

        request_message = "Here are a set of high level skills: "+string_hl_skills+". \
        These are the available objects, which can be detected:" + string_detections+ " and these could be associated colors"+ string_detection_colors +" if a color is given. \
        Moreover a set of locations "+string_locations+" is given. \
        Please select one high level skill, one object, one color and one location most fitting to this instruction: "+ nl +" \
        Only return the skill, object specifier, selected color and the location. No additional text!\
        If no location is given return 'NOT_SPECIFIED' for the location.\
        If no object is given return 'NOT_SPECIFIED' for the object. \
        If no object color is given return 'NOT_SPECIFIED' for the object color.\
        If no high level skill is given return 'NOT_SPECIFIED' for the high level skill.\
        Please list any additional information as bullet points at the end of your text."

        #print(request_message)
        response = self._client.chat.completions.create(
            messages=[
                {
                    "role": "user",
                    "content": request_message,
                }
            ],
            model="gpt-4o"
        )

        response_text = response.choices[0].message.content

        _hl_skill_flags = []
        _detections_flags = []
        _detection_color_flags = []
        _location_flags = []

        for skill in hl_skills:
            if skill in response_text:
                #print(constraint + " is in the response")
                _hl_skill_flags.append(skill)

        for detection in detections_copy:
            if detection in response_text:
                #print(goal + " is in the response")
                _detections_flags.append(detection)

        for color in detection_colors_copy:
            if color in response_text:
                #print(constraint + " is in the response")
                _detection_color_flags.append(color)

        for location in locations_copy:
            if location in response_text:
                #print(constraint + " is in the response")
                _location_flags.append(location)

        additional_information = response_text.splitlines()
        additional_information = [line.strip()[2:] for line in additional_information if line.strip().startswith("-")]

        return _hl_skill_flags, _detections_flags, _detection_color_flags, _location_flags, additional_information












# Now thinking ... 
# 1.: score: 0.2 : for action : Take off to 20 metres altitude.
# 2.: score: 0.2 : for action : Search woods2 for a person.
# 3.: score: 0.2 : for action : Search woods3 for a person.
# 4.: score: 0.2 : for action : Take off to 30 metres altitude.
# 5.: score: 0.2 : for action : Take off to 50 metres altitude.
# 6.: score: 0.0 : for action : Return to Home and Land.
# 7.: score: 0.0 : for action : Make a selfie.
# 8.: score: 0.0 : for action : Grab a beer from the nearest bar.
# 9.: score: 0.0 : for action : Drive to the moon.
# 10.: score: 0.0 : for action : Steal a sunflower.
# 11.: score: 0.0 : for action : Search woods2 for a cat.
# 12.: score: 0.0 : for action : Search woods3 for a cat.
# 13.: score: 0.0 : for action : Search urbanareas0 for a person.
# 14.: score: 0.0 : for action : Search urbanareas0 for a cat.
# Selected skill: Take off to 20 metres altitude.
# Now thinking ... 
# 1.: score: 0.5 : for action : Search woods2 for a person.
# 2.: score: 0.5 : for action : Search woods3 for a person.
# 3.: score: 0.0 : for action : Return to Home and Land.
# 4.: score: 0.0 : for action : Take off to 20 metres altitude.
# 5.: score: 0.0 : for action : Make a selfie.
# 6.: score: 0.0 : for action : Grab a beer from the nearest bar.
# 7.: score: 0.0 : for action : Drive to the moon.
# 8.: score: 0.0 : for action : Steal a sunflower.
# 9.: score: 0.0 : for action : Search woods2 for a cat.
# 10.: score: 0.0 : for action : Search woods3 for a cat.
# 11.: score: 0.0 : for action : Search urbanareas0 for a person.
# 12.: score: 0.0 : for action : Search urbanareas0 for a cat.
# 13.: score: 0.0 : for action : Take off to 30 metres altitude.
# 14.: score: 0.0 : for action : Take off to 50 metres altitude.
# Selected skill: Search woods2 for a person.
# Now thinking ... 
# 1.: score: 1.0 : for action : Search woods3 for a person.
# 2.: score: 0.0 : for action : Return to Home and Land.
# 3.: score: 0.0 : for action : Take off to 20 metres altitude.
# 4.: score: 0.0 : for action : Make a selfie.
# 5.: score: 0.0 : for action : Search woods2 for a person.
# 6.: score: 0.0 : for action : Grab a beer from the nearest bar.
# 7.: score: 0.0 : for action : Drive to the moon.
# 8.: score: 0.0 : for action : Steal a sunflower.
# 9.: score: 0.0 : for action : Search woods2 for a cat.
# 10.: score: 0.0 : for action : Search woods3 for a cat.
# 11.: score: 0.0 : for action : Search urbanareas0 for a person.
# 12.: score: 0.0 : for action : Search urbanareas0 for a cat.
# 13.: score: 0.0 : for action : Take off to 30 metres altitude.
# 14.: score: 0.0 : for action : Take off to 50 metres altitude.
# Selected skill: Search woods3 for a person.
# Now thinking ... 
# 1.: score: 0.0 : for action : Return to Home and Land.
# 2.: score: 0.0 : for action : Take off to 20 metres altitude.
# 3.: score: 0.0 : for action : Make a selfie.
# 4.: score: 0.0 : for action : Search woods2 for a person.
# 5.: score: 0.0 : for action : Grab a beer from the nearest bar.
# 6.: score: 0.0 : for action : Drive to the moon.
# 7.: score: 0.0 : for action : Steal a sunflower.
# 8.: score: 0.0 : for action : Search woods2 for a cat.
# 9.: score: 0.0 : for action : Search woods3 for a person.
# 10.: score: 0.0 : for action : Search woods3 for a cat.
# 11.: score: 0.0 : for action : Search urbanareas0 for a person.
# 12.: score: 0.0 : for action : Search urbanareas0 for a cat.
# 13.: score: 0.0 : for action : Take off to 30 metres altitude.
# 14.: score: 0.0 : for action : Take off to 50 metres altitude.
# Selected skill: Return to Home and Land.
# Now thinking ... 
# 1.: score: 1.0 : for action : Return to Home and Land.
# 2.: score: 0.0 : for action : Take off to 20 metres altitude.
# 3.: score: 0.0 : for action : Make a selfie.
# 4.: score: 0.0 : for action : Search woods2 for a person.
# 5.: score: 0.0 : for action : Grab a beer from the nearest bar.
# 6.: score: 0.0 : for action : Drive to the moon.
# 7.: score: 0.0 : for action : Steal a sunflower.
# 8.: score: 0.0 : for action : Search woods2 for a cat.
# 9.: score: 0.0 : for action : Search woods3 for a person.
# 10.: score: 0.0 : for action : Search woods3 for a cat.
# 11.: score: 0.0 : for action : Search urbanareas0 for a person.
# 12.: score: 0.0 : for action : Search urbanareas0 for a cat.
# 13.: score: 0.0 : for action : Take off to 30 metres altitude.
# 14.: score: 0.0 : for action : Take off to 50 metres altitude.
# Selected skill: Return to Home and Land.