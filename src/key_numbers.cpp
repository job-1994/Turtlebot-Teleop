#include "ncurses.h"

int main(int argc, char **argv)
	{	
		initscr();
			cbreak();
			noecho();
			while(1)
				{
					int ch = getch();
					printw("%c\n", ch);
					printw("%d\n", ch);
				}
			endwin();
	}