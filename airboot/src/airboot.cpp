#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

using namespace std;

int main( int argc, char *argv[] )
{

    FILE *fp, *ids_to_open;

    char path[1035];
    vector<string> v, nomes;
    int cont = 0, total = 0;
    std::string abrir;

    cout << "..:: AIR BOOT ::.." << endl;

    /* Open the command for reading. */
    fp = popen("/usr/bin/lsusb", "r");
    if (fp == NULL)
    {
        printf("Failed to run command\n" );
        exit(-1);
    }

    ids_to_open = fopen("/home/robo/ids_to_open.txt", "r");
    if (ids_to_open == NULL)
    {
        printf("Failed to open file '/home/robo/ids_to_open.txt' \n" );
        pclose(fp);
        exit(-1);
    }

    cout << "Buscando as IDs:" << endl;
    while (fgets(path, sizeof(path)-1, ids_to_open) != NULL)
    {
        //printf("%s", path);
        std::string s(path);

        cout << s;
        nomes.push_back(s);
        v.push_back(s.substr(0,9));
        cont ++;

     }

    cout << endl;

    /* Read the output a line at a time - output it. */
    while (fgets(path, sizeof(path)-1, fp) != NULL)
    {
        //printf("%s", path);
        std::string s(path);

        for(int i = 0; i < cont; i++){
            if (s.rfind(v[i]) != std::string::npos){

                abrir = "sudo chmod 777 /dev/bus/usb/" + s.substr(4,3) + "/" + s.substr(15,3);
                cout << nomes[i] << s << abrir << endl << endl;
                if(system(abrir.c_str()) == 0)
                    total++;
            }
        }
    }

    if(total == cont)
        cout << endl <<"\033[0;32m " << total << " de " << cont << " Abertas. \033[0m" << endl << endl;
    else
        cout << endl <<"\033[1;31m########## --> " << total << " de " << cont << " Abertas. \033[0m" << endl << endl;


    /* close */
    pclose(fp);
    fclose(ids_to_open);

    return 0;
}
