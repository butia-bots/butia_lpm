import os
os.environ["LANGCHAIN_HANDLER"] = "langchain"
import langchain
from langchain.cache import SQLiteCache
langchain.llm_cache = SQLiteCache(database_path=".langchain.db")
from langchain.python import PythonREPL


def python_repl_tool(text):
    result = python_repl.run(text)
    if result == '':
        return 'code did not print anything, you might have forgotten to print the expected output.'
    else:
        return result
    
python_repl = PythonREPL()