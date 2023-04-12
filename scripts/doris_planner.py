#!/usr/bin/env python3
import rospy
import os
os.environ["LANGCHAIN_HANDLER"] = "langchain"
import moveit_commander
import langchain
from langchain.cache import SQLiteCache
langchain.llm_cache = SQLiteCache(database_path=".langchain.db")
import butia_lpm.doris.doris_prompts as doris_prompts
from butia_lpm.doris.doris_tools import *
from butia_lpm.common.tools import *
from butia_lpm.common.agents import *
from butia_lpm.doris.doris_api import *
from langchain.agents import AgentExecutor
from langchain.agents import Tool, AgentExecutor
from langchain.llms import OpenAI
from langchain.memory import ConversationBufferMemory
from langchain.callbacks.base import CallbackManager
from langchain.callbacks.streaming_stdout import StreamingStdOutCallbackHandler
import gradio as gr


if __name__ == '__main__':

    
    llm = OpenAI(temperature=0, model_name="gpt-3.5-turbo", cache=True, streaming=True, callback_manager=CallbackManager([StreamingStdOutCallbackHandler()]), verbose=True)
    memory = ConversationBufferMemory(memory_key="chat_history", return_messages=True)
    python_repl.locals = api_locals
    tools = [
        Tool(
            name='python_repl',
            description='Useful for when you need to navigate to locations, manipulate or recognize objects by executing python code. The input to this tool should be the python code to execute. The return of this tool is anything printed by the python code, and any error message that might be raised. You should always print the output of the code. An exemple input to this tool could be: print("hello world!")',
            func=python_repl_tool
        ),
    ]
    agent = ROSAgent.from_llm_and_tools(llm=llm, tools=tools, system_message=doris_prompts.system_message, human_message=doris_prompts.human_message)
    agent_executor = AgentExecutor(memory=memory, agent=agent, tools=tools, verbose=True)
    with gr.Blocks() as demo:
        chatbot = gr.Chatbot()
        msg = gr.Textbox()
        clear = gr.Button("Clear")

        def user(user_message, history):
            return "", history + [[user_message, None]]

        def bot(history):
            bot_message = None
            while bot_message == None:
                try:
                    bot_message = agent_executor.run(input=history[-1][0])
                except ValueError as e:
                    raise e
            history[-1][1] = bot_message
            return history

        msg.submit(user, [msg, chatbot], [msg, chatbot], queue=False).then(
            bot, chatbot, chatbot
        )
        clear.click(lambda: None, None, chatbot, queue=False)

    demo.launch()
    if not mock_tools:
        moveit_commander.roscpp_shutdown()
    rospy.signal_shutdown("ChatGPT planner complete")