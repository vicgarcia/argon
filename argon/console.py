import os
from clint import textui


def clear():
    ''' clear the console '''
    os.system('clear')


def blank():
    ''' output a blank line to the console '''
    textui.puts(newline=True)


def white(text):
    ''' output white text to the console '''
    textui.puts(text)


def red(text):
    ''' output red text to the console '''
    textui.puts(textui.colored.red(text))
