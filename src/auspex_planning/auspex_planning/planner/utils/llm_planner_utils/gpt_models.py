import os
import openai
import anthropic
import google.generativeai as genai
from google.generativeai import types
import requests
from typing import Optional, Dict
import numpy as np

class MultiLLMClient:
    def __init__(self, provider: str, model: str):
        """Initialize the LLM client with a provider and model."""
        self.provider = provider.lower()
        self.model = model
        self.api_key = os.getenv(f"{self.provider.upper()}_API_KEY")
        self.client = None
        if not self.api_key:
            raise ValueError(f'API key for {self.provider} not found!')

    def query_openai(self, prompt: str, max_tokens: int = 100) -> dict:
        """Query OpenAI's API"""
        try:
            if self.client is None:
                self.client = openai.OpenAI(api_key=self.api_key)
            response = self.client.chat.completions.create(
                model=self.model,
                messages=prompt,
                max_tokens=min(max_tokens, 16384),
                temperature=0,
                logprobs=True,
                top_logprobs=1
            )
            return {
                "response": response.choices[0].message.content,
                "logprobs": response.choices[0].logprobs.content[0].top_logprobs
            }

        except Exception as e:
            print(f"OpenAI API error: {e}")
            return {"error": str(e)}

    def query_deepseek(self, prompt: str, max_tokens: int = 100) -> Dict:
        """Query DeepSeek API"""
        try:
            if self.client is None:
                self.client = openai.OpenAI(api_key=self.api_key, base_url="https://api.deepseek.com")

            response = self.client.chat.completions.create(
                model=self.model,
                messages=prompt,
                max_tokens=min(max_tokens, 8192),
                temperature=0,
                logprobs=True,
                top_logprobs=1
            )
            return {
                "response": response.choices[0].message.content,
                "logprobs": response.choices[0].logprobs.content[0].top_logprobs
            }

        except Exception as e:
            return {"error": str(e)}

    def query_anthropic(self, prompt: str, max_tokens: int = 100) -> Dict:
        """Query Anthropic's Claude API"""
        try:
            if self.client is None:
                self.client = anthropic.Anthropic(api_key=self.api_key)

            response = self.client.messages.create(
                model=self.model,#"claude-3-7-sonnet-20250219"
                max_tokens=min(max_tokens, 4096),
                messages=prompt
            )
            return {
                "response": response.content[0].text,
                "logprobs": None
            }
        except Exception as e:
            return {"error": str(e)}

    def query_gemini(self, prompt: str, max_tokens: int = 100) -> Dict:
        """Query Google Gemini API."""
        try:
            # Configure once and cache the model
            if self.clients['gemini'] is None:
                self.client = genai.Client(api_key=self.api_key)

            response = self.client.models.generate_content(
                model=self.model, #"gemini-2.0-flash"
                contents=prompt[1]['content'],
                config=types.GenerateContentConfig(
                    max_output_tokens=max_tokens,
                    temperature=0,
                    system_instruction=prompt[0]['content'],
                )
            )

            return {
                "response": response.text,
                "logprobs": None
            }
        except Exception as e:
            return {"error": str(e)}

    def query(self, prompt: str, max_tokens: int = 100) -> Dict:
        """Route query to the appropriate provider based on initialization."""
        if self.provider == "openai":
            return self.query_openai([{"role":"system","content":prompt}], max_tokens)
        elif self.provider == "anthropic":
            return self.query_anthropic([{"role": "user","content": prompt}], max_tokens)
        elif self.provider == "deepseek":
            return self.query_deepseek([{"role":"system","content":prompt}], max_tokens)
        elif self.provider == "gemini":
            return self.query_gemini(prompt, max_tokens)
        else:
            return {"error": "Unsupported provider"}
