src=*.cpp
rasterizer_c:
	gcc -o rasterizer $(src)  -lm

rasterizer_cpp:
	g++ -std=c++11 -o rasterizer $(src) 

clean:
	rm -rf *.ppm *.png ./rasterizer
