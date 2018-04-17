#include <iostream>
#include <fstream>

class Register
{
public:
	Register()
	{
		std::string filename="/etc/openert";
		std::ifstream f(filename);
    	
    	if(f.good())
    	{
    		printf("Cannot re-register!!!\n");
    		exit(0);
    	}	
    	else
    	{
    		try
    		{
				system(("sudo touch "+filename+"&& sudo chmod 666 "+filename+"&& echo \"ll\">> "+filename).c_str());
			    printf("Register Done!!!\n");
			}
			catch(...)
			{
				printf("Cannot register! Contact NTU IOT Lab!\n");
				exit(0);
			}
		}
	}
};


int main ()
{
	Register reg;
	return 0;
}
