import pyttsx

class speech:

	def __init__(self):
		
		self.engine = pyttsx.init()


	def talk(self, text, voice = 'english-us'):
		
		self.engine.setProperty('voice', voice)
		self.engine.say(text)
		self.engine.runAndWait()
