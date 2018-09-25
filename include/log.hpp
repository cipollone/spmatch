
#pragma once

#include <iostream>
#include <string>
#include <sstream>
#include <stdexcept>

#include "params.hpp"


using std::string;


/*******************************************
* > sStr()                                 *
* String stream convertion.                *
*                                          *
* Args:                                    *
*   obj (T): an object with << overloading *
*                                          *
* Returns:                                 *
*   (string): the string representation    *
*******************************************/
template<typename T>
string sStr(const T& obj) {
	std::ostringstream stream;
	stream << obj;
	return stream.str();
}


/*************************************************************************
* > logMsg()                                                             *
* Log messages. Prints to stdout the message if the current log level is *
* above or equal 'level'.                                                *
* NOTE: using std::flush will slow down the process                      *
*                                                                        *
* Args:                                                                  *
*   message (string): the message to print.                              *
*   level (int): the log level of this message.                          *
*   end (char): separator character; defaults to '\n'.                   *
*   flush (bool): if true, std::flush is sent. Defaults to false         *
*************************************************************************/
inline void logMsg(const string& message, int level, char end='\n',
		bool flush=false) {
	if (params.LOG >= level) {
		if (flush) {
			std::cout << message << end << std::flush;
		} else {
			std::cout << message << end;
		}
	}
}
