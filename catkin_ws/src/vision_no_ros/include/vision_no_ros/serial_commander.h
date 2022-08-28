#ifndef SERIAL_COMMANDER_H
#define SERIAL_COMMANDER_H

static int command=-1;

void set_command(int new_command){
    //std::cout << "enter new command"<< std::endl;
    //std::cin >> command;
    command=new_command;
}



int get_command(){
    return command;
}
#endif
