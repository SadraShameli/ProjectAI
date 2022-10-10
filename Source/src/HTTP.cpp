#include "HTTP.h"

/*
#include <HTTPServer.h>
#include <HTTPRequestHandler.h>
#include <HTTPRequestHandlerFactory.h>
#include <HTTPServerParams.h>
#include <HTTPServerRequest.h>
#include <HTTPServerResponse.h>
#include <HTTPServerParams.h>

using namespace Poco::Net;
using namespace Poco::Util;

namespace ProjectAI
{
    void HTTPServer::Start()
    {
        Poco::UInt16 port = 9999;
        HTTPServerParams *pParams = new HTTPServerParams;
        pParams->setMaxQueued(100);
        pParams->setMaxThreads(16);
        ServerSocket svs(port); // set-up a server socket
        HTTPServer srv(new MyRequestHandlerFactory(), svs, pParams);
        // start the HTTPServer
        srv.start();
        waitForTerminationRequest();
        // Stop the HTTPServer
        srv.stop();
    }
} */