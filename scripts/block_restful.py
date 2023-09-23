import requests

prompt = "hellow world c d"
api_base = "http://192.168.50.66:8080/prompt_len"
pdata = {
    "prompt": prompt
}

response_str = requests.post(api_base, headers={"Content-Type": "application/json"}, json=pdata).json()
print response_str["token_count"]
