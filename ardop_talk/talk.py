import pyttsx

engine = pyttsx.init()
voices = engine.getProperty('voices')

engine.setProperty('voice', 'english-us')  # changes the voice
engine.say('This is a person. Detected with 100 percent accuracy')
engine.runAndWait()