#!/usr/bin/env python

from __future__ import print_function

import json
import os
import re
import time

import requests

import utils as ut


class ChatGPT:
    def __init__(self, configs):
        self.json_dict = None
        self.environment = None
        self.credentials = json.load(open('c.json'), strict=False)
        self.messages = []
        self.max_token_length = 15000  # 4000
        self.max_completion_length = 2000  # 1300
        self.last_response = None
        self.last_response_raw = None
        self.query = ''
        self.instruction = ''
        # load prompt file
        fp_system = os.path.join(configs["system"])
        with open(fp_system) as fl:
            data = fl.read()
        self.system_message = {"role": "system", "content": data}

        for prompt_name in configs["prompt_load_order"]:
            fp_prompt = os.path.join(prompt_name)
            with open(fp_prompt) as fl:
                data = fl.read()
            data_split = re.split(r'\[user]\n|\[assistant]\n', data)
            data_split = [item for item in data_split if len(item) != 0]
            # it starts with user and ends with system
            assert len(data_split) % 2 == 0
            for i, item in enumerate(data_split):
                if i % 2 == 0:
                    self.messages.append({"sender": "user", "text": item})
                else:
                    self.messages.append({"sender": "assistant", "text": item})
        fp_query = os.path.join(configs["query"])
        with open(fp_query) as fl:
            self.query = fl.read()

    def create_prompt(self):
        prompt = [self.system_message]
        for message in self.messages:
            prompt.append(
                {"role": message['sender'], "content": message['text']})
        prompt_content = ""
        for message in prompt:
            prompt_content += message["content"]
        prompt_len = ut.get_token_count(prompt_content)
        print('prompt length: ' + str(prompt_len))
        if ut.get_token_count(prompt_content) > self.max_token_length - self.max_completion_length:
            print('prompt too long. truncated.')
            # truncate the prompt by removing the oldest two messages
            self.messages = self.messages[2:]
            prompt = self.create_prompt()
        return prompt

    @staticmethod
    def extract_json_part(text_input):
        # because the json part is in the middle of the text, we need to extract it.
        # json part is between ```python and ```.
        # skip if there is no json part
        if text_input.find('```python') == -1:
            return text_input
        text_json = text_input[text_input.find(
            '```python') + len('```python'):text_input.find('\n```')]
        text_json.replace('```', '')
        return text_json

    def generate(self, message, environment, is_user_feedback=False):
        if is_user_feedback:
            self.messages.append({'sender': 'user', 'text': message})
        else:
            text_base = self.query
            if text_base.find('[ENVIRONMENT]') != -1:
                text_base = text_base.replace(
                    '[ENVIRONMENT]', json.dumps(environment))
            if text_base.find('[INSTRUCTION]') != -1:
                text_base = text_base.replace('[INSTRUCTION]', message)
                self.instruction = text_base
            self.messages.append({'sender': 'user', 'text': text_base})

        response = self.get_gpt_response()
        text_response = response['choices'][0].message.content
        self.last_response_raw = text_response
        self.messages.append(
            {"sender": "assistant", "text": self.last_response_raw})
        # analyze the response
        self.last_response = text_response
        self.last_response = self.extract_json_part(self.last_response)
        self.last_response = self.last_response.replace("'", "\"")
        try:
            self.json_dict = json.loads(self.last_response, strict=False)
            self.environment = self.json_dict["environment_after"]
        except Exception as e:
            print(e)
            self.json_dict = None
            return None
        return self.json_dict

    def get_gpt_response(self):
        headers = {"Authorization": "Bearer " + self.credentials["api_key"]}
        fp_prompt = os.path.join("", "tiago_prompt_pickplace.txt")
        with open(fp_prompt) as fl:
            sys_prompt = fl.read()
        data = {
            "model": "gpt-3.5-turbo-16k",
            "temperature": 2.0,
            "max_tokens": self.max_completion_length,
            "top_p": 0.5,
            "frequency_penalty": 0.0,
            "presence_penalty": 0.0,
            'messages': [
                {"role": "system", "content": sys_prompt},
                {"role": "user", "content": str("hello")}
            ]
        }
        response_str = requests.post(self.credentials["api_base"], headers=headers, json=data)
        print(response_str.text)
        resp_msg = response_str.json()['choices'][0]['message']['content']
        # print resp_msg
        json_msg = json.loads(resp_msg, strict=True)
        return json_msg

        # time_str = time.strftime("%Y%m%d-%H%M%S")
        # json_file_name = "../json_files/result_" + time_str + ".json"
        # with open(json_file_name, 'w') as fp:
        #     json.dump(json_msg, fp)


if __name__ == "__main__":
    global_configs = {
        "system": "block_p01_system.txt",
        "prompt_load_order": [
            "block_p02_role.txt",
            "block_p03_env.txt",
            "block_p04_function.txt",
            "block_p05_output.txt",
            "block_p06_example.txt"
        ],
        "query": "block_p07_query.txt",
    }
    gpt = ChatGPT(global_configs)
    dir_name = "out_task_planning_gpt-3.5-turbo-16k_temp=2.0"
    waittime_sec = 30
    max_trial = 5
    time_api_called = time.time() - waittime_sec

    input_json = {
        "environment": {
            "assets": [
                "<Joy's table>",
                "<Jack's table>",
            ],
            "blocks": [
                "<block 658>"
            ],
            "states": {
                "<block 658>": [
                    "ON(<Jack's table>)"
                ]
            }
        },
        "instructions": [
            "place block 658 on Joy's table"
        ],
    }

    # call until break
    while True:
        # if api is called within 60 seconds, wait
        current_time = time.time()
        if current_time - time_api_called < waittime_sec:
            print("waiting for " + str(waittime_sec - (current_time - time_api_called)) + " seconds...")
            time.sleep(waittime_sec - (current_time - time_api_called))
        text = gpt.generate(input_json["instructions"][0], input_json["environment"], is_user_feedback=False)
        time_api_called = time.time()
        if text is not None:
            break
        else:
            print("api call failed. retrying...")
            current_time = time.time()
            if current_time - time_api_called < waittime_sec:
                print("waiting for " + str(waittime_sec - (current_time - time_api_called)) + " seconds...")
                time.sleep(waittime_sec - (current_time - time_api_called))

            format_error = "Your return cannot be interpreted as a valid json dictionary. Please reformat your response."
            text = gpt.generate(format_error, current_time, is_user_feedback=True)
            break
    print(text)

    print("c")
