#include "bmp_manager.h"
#include <iostream>

TheBMP::~TheBMP()
{
	delete []image;
}

void TheBMP::readin(LPCWSTR filename)
{
   HANDLE hfile;
   DWORD written;
  // hfile = CreateFile(filename,GENERIC_READ,FILE_SHARE_READ,NULL,OPEN_EXISTING,NULL,NULL);
   hfile = CreateFileW (filename,GENERIC_READ,FILE_SHARE_READ,NULL,OPEN_EXISTING,NULL,NULL);
   ReadFile(hfile,&bfh,sizeof(bfh),&written,NULL);
   ReadFile(hfile,&bih,sizeof(bih),&written,NULL);
   // Read image
   int imagesize = bih.biWidth*bih.biHeight; // Helps you allocate memory for the image

   std::cout<<std::endl;
   std::cout<<"width:"<<bih.biWidth<<std::endl;
   std::cout<<"height:"<<bih.biHeight<<std::endl;
   std::cout<<std::endl;
   image = new RGBTRIPLE[imagesize]; // Create a new image (I'm creating an array during runtime)  
   ReadFile(hfile,image,imagesize*sizeof(RGBTRIPLE),&written,NULL); // Reads it off the disk  
   // Close source file
   CloseHandle(hfile);
}


RGBTRIPLE TheBMP::getpixel(int x,int y)
{
	return image[(y)*bih.biWidth+x];
}