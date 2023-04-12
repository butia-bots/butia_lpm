#!/usr/bin/env python3
import os
os.environ["LANGCHAIN_HANDLER"] = "langchain"
#import openai
from typing import List, Any, Optional
import langchain
from langchain.cache import SQLiteCache
langchain.llm_cache = SQLiteCache(database_path=".langchain.db")
from langchain.agents.conversational_chat.prompt import PREFIX, SUFFIX
from langchain.schema import AIMessage
from langchain.agents import ConversationalChatAgent
from langchain.agents.conversational_chat.base import AgentOutputParser

class ROSConversationalChatOutputParser(AgentOutputParser):
    def parse(self, text: str) -> Any:
        # Add ```json back to the text, since we manually added it as an AIMessage in create_prompt
        return super().parse(f"```json{text.split('```')[0]}")


class ROSAgent(ConversationalChatAgent):
    def _construct_scratchpad(
        self, intermediate_steps
    ):
        thoughts = super()._construct_scratchpad(intermediate_steps)
        # Manually append an AIMessage with ```json to better guide the LLM towards responding with only one action and no prose.
        thoughts.append(AIMessage(content="```json"))
        return thoughts

    @classmethod
    def create_prompt(
        cls,
        tools,
        system_message: str = PREFIX,
        human_message: str = SUFFIX,
        input_variables: Optional[List[str]] = None,
        output_parser = None,
    ):
        return super().create_prompt(
            tools,
            system_message,
            human_message,
            input_variables,
            output_parser or ROSConversationalChatOutputParser(),
        )

    @classmethod
    def from_llm_and_tools(
        cls,
        llm,
        tools,
        callback_manager = None,
        system_message: str = PREFIX,
        human_message: str = SUFFIX,
        input_variables: Optional[List[str]] = None,
        output_parser = None,
        **kwargs: Any,
    ):
        return super().from_llm_and_tools(
            llm,
            tools,
            callback_manager,
            system_message,
            human_message,
            input_variables,
            output_parser or ROSConversationalChatOutputParser(),
            **kwargs,
        )