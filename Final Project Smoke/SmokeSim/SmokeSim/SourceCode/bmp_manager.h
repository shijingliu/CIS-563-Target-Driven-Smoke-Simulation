#pragma once

#include <windows.h>

class TheBMP{
public:
	~TheBMP();
	BITMAPFILEHEADER bfh;
    BITMAPINFOHEADER bih;
    RGBTRIPLE *image;
//////////////////////////////
	void readin(LPCWSTR filename);
	RGBTRIPLE getpixel(int x,int y);
//////////////////////////////
};