CC=g++ -std=c++14
LIBS+=`pkg-config opencv --libs --cflags`

camera_calibration: camera_calibration.cc
	$(CC) -o camera_calibration camera_calibration.cc $(LIBS) 

all: camera_calibration capture_chessboards

capture_chessboards: capture_chessboards.cc
	$(CC) -o $@ $< $(LIBS) 
