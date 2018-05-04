import pyttsx
import inflect

def speak(arg, voice = "english-us"):

    engine = pyttsx.init()
    engine.setProperty('voice', voice)
    rate = engine.getProperty('rate')
    engine.setProperty('rate', rate-5)
    engine.say(arg)
    engine.say("   ")
    engine.runAndWait()

def numToWords(arg):

	p = inflect.engine()
	return p.number_to_words(arg)

