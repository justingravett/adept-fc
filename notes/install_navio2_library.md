Note: you must have the latest Raspbian distribution from Emlid installed. 

# Installing the Navio2 device driver library
1. Clone the Navio2 [git repo](https://github.com/emlid/Navio2.git) into the Pi's home directory. 
2. Navigate to `C++/Navio/`
3. Type `make all`

Another Note: If you have trouble compiling `adept-fc`, check the path's in our main MakeFile and ensure you're Navio2 library is in the expected location. 
