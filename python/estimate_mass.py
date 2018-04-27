density = 1.04
volume = float(raw_input("Enter volume: "))
infill = float(raw_input("Enter infill: "))
servos = int(raw_input("Enter number of servos: "))
print 'Mass: %f kg'%((volume*density*infill*0.001) + (0.065*servos))
