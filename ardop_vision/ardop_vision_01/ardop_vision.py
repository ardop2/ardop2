'''
./darknet detector demo cfg/coco.data cfg/yolov3.cfg weights/yolov3.weights -c 0
./darknet detect cfg/yolov3.cfg weights/yolov3.weights data/0.jpg
'''

import os
import sys
from subprocess import Popen, PIPE
import cv2

# Get this path
MAIN_PATH = os.path.abspath(os.path.dirname(__file__))

# Append sys paths to find the modules
sys.path.append(os.path.abspath(os.path.dirname(__file__))+'/kinect/wrappers/python/')
sys.path.append(os.path.abspath(os.path.dirname(__file__))+'/darknet')

# Import darknet
from darknet import darknet
DARKNET_PATH = darknet.get_path()

# Import kinect
import demo_cv2_sync as kinect
KINECT_PATH = kinect.get_path()

SAMPLES = 5
YOLO_RESULTS = [[('person', 0.9979779124259949, (430.2414245605469, 256.23455810546875, 293.356201171875, 421.81744384765625)), ('person', 0.9958539009094238, (110.78553009033203, 246.41329956054688, 222.70040893554688, 414.9528503417969)), ('chair', 0.9619964361190796, (243.24038696289062, 373.5849304199219, 121.11754608154297, 216.1255645751953)), ('chair', 0.7783901691436768, (227.4122314453125, 282.9215393066406, 58.243324279785156, 84.26954650878906)), ('bottle', 0.7504300475120544, (334.3214416503906, 249.25941467285156, 40.41710662841797, 143.8472137451172))], [('person', 0.9964104294776917, (109.7206039428711, 246.34127807617188, 222.24029541015625, 409.43585205078125)), ('person', 0.9963383078575134, (429.54510498046875, 257.0513610839844, 295.41485595703125, 422.8762512207031)), ('chair', 0.975904643535614, (251.26771545410156, 372.4876403808594, 130.1700439453125, 225.3291473388672)), ('chair', 0.8181812763214111, (226.22418212890625, 281.433349609375, 60.33164978027344, 91.47119140625)), ('bottle', 0.7066519856452942, (333.17010498046875, 249.948486328125, 41.98982238769531, 142.55160522460938))], [('person', 0.9976082444190979, (429.6758728027344, 257.07879638671875, 298.4423522949219, 423.77685546875)), ('person', 0.9964763522148132, (109.1976318359375, 247.78024291992188, 215.47479248046875, 428.257568359375)), ('chair', 0.9727608561515808, (251.462890625, 371.9004821777344, 129.00103759765625, 226.414794921875)), ('bottle', 0.9189388155937195, (333.1492919921875, 248.7589874267578, 41.534915924072266, 142.73434448242188)), ('chair', 0.756619393825531, (226.6789093017578, 282.6851806640625, 58.426002502441406, 83.09564208984375))], [('person', 0.9975007772445679, (430.0997314453125, 258.1213684082031, 289.24859619140625, 423.95672607421875)), ('person', 0.9944122433662415, (112.43948364257812, 247.2559814453125, 222.07406616210938, 413.663818359375)), ('chair', 0.9807090163230896, (251.10714721679688, 372.2397155761719, 129.40536499023438, 224.5850372314453)), ('bottle', 0.8030130863189697, (333.7650146484375, 249.5998992919922, 41.624183654785156, 140.5918731689453)), ('chair', 0.7088439464569092, (226.6305694580078, 283.0061950683594, 58.26612091064453, 85.14901733398438))], [('person', 0.9974158406257629, (428.24969482421875, 256.9350891113281, 295.931640625, 422.32208251953125)), ('person', 0.9965169429779053, (109.7705078125, 245.70785522460938, 207.58016967773438, 420.3697509765625)), ('chair', 0.9863286018371582, (251.12046813964844, 371.0027160644531, 132.63296508789062, 222.1945037841797)), ('chair', 0.8340767025947571, (226.37979125976562, 283.73236083984375, 59.04801940917969, 84.39765167236328)), ('bottle', 0.7702242136001587, (333.07916259765625, 248.91766357421875, 41.15211868286133, 144.50564575195312)), ('chair', 0.5005460977554321, (79.0929946899414, 337.0498046875, 159.0775146484375, 286.9723205566406))]]

# List of objects
LABELS = None

# Formatting
HEADER = '\033[95m'
OKBLUE = '\033[94m'
OKGREEN = '\033[92m'
WARNING = '\033[93m'
FAIL = '\033[91m'
ENDC = '\033[0m'
BOLD = '\033[1m'
UNDERLINE = '\033[4m'

def cd_main():
	os.chdir(MAIN_PATH)

def cd_darknet():
	os.chdir(DARKNET_PATH)

def cd_kinect():
	os.chdir(KINECT_PATH)

def display_rgbd_map(rgb, depth):
	alpha = 0.5

	depth = cv2.cvtColor(depth, cv2.COLOR_GRAY2BGR)
	depth = cv2.cvtColor(depth, cv2.COLOR_HSV2BGR)
	cropped = rgb[30:480, 8:600]
	resized_image = cv2.resize(cropped, (640, 480))

	temp = cv2.addWeighted(depth, alpha, resized_image, 1-alpha, 0) 
	cv2.imshow('RGBD Map', temp)

def calibrate_image(rgb):
	cropped = rgb[30:480, 8:600]
	resized_image = cv2.resize(cropped, (640, 480))
	return resized_image

def generate_kinect_images():
	print WARNING + BOLD + 'Generating images from Kinect...' + ENDC
	cd_kinect()

	for i in range(0, SAMPLES):
		rgb = calibrate_image(kinect.get_rgb())
		depth = kinect.get_depth()

		rgb_path = '%s/rgb/%d.jpg'%(MAIN_PATH, i)
		print rgb_path

		depth_path = '%s/depth/%d.jpg'%(MAIN_PATH, i)
		print depth_path

		# cv2.imwrite(rgb_path, rgb)
		# cv2.imwrite(depth_path, depth)

	print BOLD + OKGREEN + 'Images Saved\n' + ENDC

def display_images():
	cd_kinect()
	while (1):
		rgb = kinect.get_rgb()
		rgb_ci = calibrate_image(kinect.get_rgb())
		cv2.imshow('RGB', rgb_ci)

		depth = kinect.get_depth()
		depth_map = cv2.cvtColor(depth, cv2.COLOR_GRAY2BGR)
		depth_map = cv2.cvtColor(depth_map, cv2.COLOR_HSV2BGR)
		cv2.imshow('Depth', depth_map)

		display_rgbd_map(rgb, depth)

		# Exit on pressing spacebar 
		if cv2.waitKey(1) & 0xFF == ord(' '): 
			break

	cv2.destroyAllWindows()

def load_labels():
	global LABELS
	cd_main()
	with open('labels.txt') as f:
		LABELS = [i.split('\n')[0] for i in f.readlines()]
	print LABELS

def load_graph():
	print WARNING + BOLD + '\nLoading YOLO network...' + ENDC
	cd_darknet()
	darknet.load_yolo(3)
	print BOLD + OKGREEN + 'Done' + ENDC

def get_images_list(folder):
	images_list = []
	for root, dirs, files in os.walk(folder):
		for file in files:
			images_list.append(os.path.abspath(os.path.join(root, file)))
	return images_list

def detect_objects():
	print WARNING + BOLD + '\nRunning YOLO network...' + ENDC
	
	cd_main()
	rgb_images = get_images_list('./rgb')

	results = []

	cd_darknet()
	for rgb_image in rgb_images:
		results.append(darknet.yolo_detect(rgb_image))

	print BOLD + OKGREEN + 'Done' + ENDC

	return results

def detect_objects_new():
	# Get list of RGB images to run detection
	cd_main()
	rgb_images = get_images_list('./rgb')

	# Detect objects
	print WARNING + BOLD + '\nRunning YOLO network...' + ENDC
	results = []
	cd_darknet()

	for rgb_image in rgb_images:
		results.append(darknet.yolo_detect(rgb_image).append(rgb_image))
	
	print BOLD + OKGREEN + 'Done' + ENDC

	return results

def display_detections(results):
	cd_main()
	images = get_images_list('./rgb')

	print images

	for result, image in zip(results, images):

		img = cv2.imread(image)
		# cv2.imshow('Input', img)

		for obj in result:
			# TODO: Reject objects
			bb = [int(i) for i in obj[2]]
			# if not obj[0] == 'bottle':
			# 	continue
			print obj[0], bb
			# cv2.rectangle(img, (bb[0], bb[1]), (bb[0]+bb[2], bb[1]+bb[3]), (0,255,0), 3)
			cv2.circle(img, (bb[0], bb[1]), 10, (0,0,255), -1)

			# break
			
		cv2.imshow('Input', img)
		break

def display_detections_new(path, bb):
	cd_main()
	img = cv2.imread(path)

	for obj in result:
		bb = [int(i) for i in obj[2]]
		print obj[0], bb

		# cv2.rectangle(img, (bb[0], bb[1]), (bb[0]+bb[2], bb[1]+bb[3]), (0,255,0), 3)
		cv2.circle(img, (bb[0], bb[1]), 10, (0,0,255), -1)
		cv2.imshow('Output', img)

def process_results(results):
	for result in results:
		for label, confidence, box, path in result:
			# Select objects from labels.txt file
			if not label in LABELS:
				continue

			display_detections_new()

def run():
	# Load YOLO
	load_graph()

	# Get RGBD samples from Kinect
	# generate_kinect_images()

	# Run YOLO and get predictions
	print detect_objects()

	# Process results

	# Close
	# cv2.waitKey(0)
	# cv2.destroyAllWindows()

if __name__ == '__main__':
	# run()
	# display_images()
	# detect_objects()
	display_detections(YOLO_RESULTS)
	# process_results(YOLO_RESULTS)
	# load_labels()

	cv2.waitKey(0)
	cv2.destroyAllWindows()
