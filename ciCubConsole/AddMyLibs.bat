robocopy ..\..\BoardLibrary\Debug\ ..\Debug BoardLibrary.dll
robocopy ..\..\BoardLibrary\Debug\ ..\Debug BoardLibrary.lib

robocopy ..\..\BoardLibrary\library\include\ ..\include BoardLibrary.h
robocopy ..\..\BoardLibrary\library\include\ ..\include phil_board.h
robocopy ..\..\BoardLibrary\library\include\ ..\include utils.h

robocopy ..\..\BoardLibrary\library\src\ ..\src utils.c
