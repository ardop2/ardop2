fl = []

MODE = 1

if MODE==1:
	# MASSIVE one-liner!!!
	for i in range(0,3): fl += [float(x)/(10.0**5) for ind, x in enumerate(raw_input("Enter Row %d: "%(i+1)).split("|")[1].split(' ')) if 4 > ind > i]

	output = '''<inertia
					ixx="%.10f"
					ixy="%.10f"
					ixz="%.10f"
					iyy="%.10f"
					iyz="%.10f"
					izz="%.10f" /> '''

	print '\nCopy Pasta into URDF:'
	print '-'*80
	print output%(fl[0], fl[1], fl[2], fl[3], fl[4], fl[5])
	print '-'*80

if MODE==2:
	# MASSIVE one-liner!!!
	fl += [float(x)/(10.0) for ind, x in enumerate(raw_input("Enter COM: ").split(' '))]

	output = '''xyz="%.10f %.10f %.10f" '''

	print '\nCopy Pasta into URDF:'
	print '-'*80
	print output%(fl[0], fl[1], fl[2])
	print '-'*80
