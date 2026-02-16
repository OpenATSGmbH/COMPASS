#include "traced_assert.h"

#include "msghandler.h"

#include <cstdlib>
#include <iostream>

namespace compass_assert
{
    void assertion_failed(char const * expr, 
                                 char const * function,
                                 char const * file, 
                                 long line,
                                 const boost::stacktrace::stacktrace& stack_trace,
                                 bool expr_is_message)
    {
        std::stringstream ss;
        ss << stack_trace;

        //compile message
        msghandler::Message msg;
        msg.severity    = msghandler::Severity::Abort;
        msg.content     = expr_is_message ? std::string(expr) : "Assertion '" + std::string(expr) + "' failed";
        msg.file        = std::string(file);
        msg.line        = (int)line;
        msg.stack_trace = ss.str();
        msg.user_level  = false;

        //log assert msg
        // Check if log4cpp root category exists
        try 
        {
            log4cpp::Category& root = log4cpp::Category::getRoot();
            // If we get here, log4cpp is initialized
            msghandler::MessageHandler::log(logerr.getStream(), msg);
        }
        catch (...) // failed, switch to cerr
        {
            bool aborting = true;
            bool show_st = aborting && !msg.stack_trace.empty();

            std::cerr << "Encountered critical error" << (aborting ? ", going into shutdown" : "")
                      << "\n"
                      << "\n"
                      << "Error:       " << (msg.content.empty() ? "Unknown error" : msg.content)
                      << "\n"
                      << "Error Code:  " << msg.err_code << "\n"
                      << "Component:   " << (msg.component.empty() ? "Unknown" : msg.component)
                      << "\n"
                      << "File:        " << msg.file << "\n"
                      << "Line:        " << msg.line << "\n"
                      << (show_st ? "\n" : "") << (show_st ? msg.stack_trace : "")
                      << (show_st ? "\n" : "") << (aborting ? "Aborting..." : "");
        }

        //then abort
        std::abort();
    }
}