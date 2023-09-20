#!/usr/bin/env python

from __future__ import print_function

import json
import os
import re
import ast


global_configs = {
    "system": "block_pmt_01_system.txt",
    "prompt_load_order": [
        "block_pmt_02_role.txt",
        "block_pmt_03_function.txt",
        "block_pmt_04_env.txt",
        "block_pmt_05_output.txt",
        "block_pmt_06_example.txt"
    ],
    "query": "block_pmt_07_query.txt",
}


class ChatGPT:
    def __init__(self, configs):
        self.credentials = configs["credentials"]
        self.messages = []
        self.max_token_length = 15000  # 4000
        self.max_completion_length = 2000  # 1300
        self.last_response = None
        self.last_response_raw = None
        self.query = ''
        self.instruction = ''
        # load prompt file
        fp_system = os.path.join(configs["system"])
        with open(fp_system) as f:
            data = f.read()
        self.system_message = {"role": "system", "content": data}

        for prompt_name in configs["prompt_load_order"]:
            fp_prompt = os.path.join(prompt_name)
            with open(fp_prompt) as f:
                data = f.read()
            data_spilit = re.split(r'\[user\]\n|\[assistant\]\n', data)
            data_spilit = [item for item in data_spilit if len(item) != 0]
            # it start with user and ends with system
            assert len(data_spilit) % 2 == 0
            for i, item in enumerate(data_spilit):
                if i % 2 == 0:
                    self.messages.append({"sender": "user", "text": item})
                else:
                    self.messages.append({"sender": "assistant", "text": item})
        fp_query = os.path.join(configs["query"])
        with open(fp_query) as f:
            self.query = f.read()

    def create_prompt(self):
        prompt = []
        prompt.append(self.system_message)
        for message in self.messages:
            prompt.append(
                {"role": message['sender'], "content": message['text']})
        prompt_content = ""
        for message in prompt:
            prompt_content += message["content"]

        print('prompt length: ' + str(len(ast.encode(prompt_content))))
        if len(enc.encode(prompt_content)) > self.max_token_length - \
                self.max_completion_length:
            print('prompt too long. truncated.')
            # truncate the prompt by removing the oldest two messages
            self.messages = self.messages[2:]
            prompt = self.create_prompt()
        return prompt

    def extract_json_part(self, text):
        # because the json part is in the middle of the text, we need to extract it.
        # json part is between ```python and ```.
        # skip if there is no json part
        if text.find('```python') == -1:
            return text
        text_json = text[text.find(
            '```python') + len('```python'):text.find('\n```')]
        text_json.replace('```', '')
        return text_json

    def generate(self, message, environment, is_user_feedback=False):
        if is_user_feedback:
            self.messages.append({'sender': 'user',
                                  'text': message})
        else:
            text_base = self.query
            if text_base.find('[ENVIRONMENT]') != -1:
                text_base = text_base.replace(
                    '[ENVIRONMENT]', json.dumps(environment))
            if text_base.find('[INSTRUCTION]') != -1:
                text_base = text_base.replace('[INSTRUCTION]', message)
                self.instruction = text_base
            self.messages.append({'sender': 'user', 'text': text_base})

        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo-16k",
            messages=self.create_prompt(),
            temperature=2.0,
            max_tokens=self.max_completion_length,
            top_p=0.5,
            frequency_penalty=0.0,
            presence_penalty=0.0)
        text = response['choices'][0].message.content
        self.last_response_raw = text
        self.messages.append(
            {"sender": "assistant", "text": self.last_response_raw})
        # analyze the response
        self.last_response = text
        self.last_response = self.extract_json_part(self.last_response)
        self.last_response = self.last_response.replace("'", "\"")
        try:
            self.json_dict = json.loads(self.last_response, strict=False)
            self.environment = self.json_dict["environment_after"]
        except BaseException:
            self.json_dict = None
            return None
        return self.json_dict
