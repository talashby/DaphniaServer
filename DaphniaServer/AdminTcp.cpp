
#include "AdminTcp.h"
#include "AdminProtocol.h"
#include "ParallelPhysics.h"

#undef UNICODE
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdlib.h>
#include <stdio.h>
// Need to link with Ws2_32.lib
#pragma comment (lib, "Ws2_32.lib")
// #pragma comment (lib, "Mswsock.lib")

#define DEFAULT_BUFLEN 512

namespace PPh
{

void AdminTcpThread()
{
	while (true)
	{
		WSADATA wsaData;
		int iResult;

		SOCKET ListenSocket = INVALID_SOCKET;
		SOCKET ClientSocket = INVALID_SOCKET;

		struct addrinfo *result = NULL;
		struct addrinfo hints;

		int iSendResult;
		char recvbuf[DEFAULT_BUFLEN];
		int recvbuflen = DEFAULT_BUFLEN;

		// Initialize Winsock
		iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
		if (iResult != 0) {
			printf("AdminTcp WSAStartup failed with error: %d\n", iResult);
			return;
		}

		ZeroMemory(&hints, sizeof(hints));
		hints.ai_family = AF_INET;
		hints.ai_socktype = SOCK_STREAM;
		hints.ai_protocol = IPPROTO_TCP;
		hints.ai_flags = AI_PASSIVE;

		// Resolve the server address and port
		iResult = getaddrinfo(NULL, ADMIN_TCP_PORT_STR, &hints, &result);
		if (iResult != 0) {
			printf("AdminTcp getaddrinfo failed with error: %d\n", iResult);
			WSACleanup();
			return;
		}

		// Create a SOCKET for connecting to server
		ListenSocket = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
		if (ListenSocket == INVALID_SOCKET) {
			printf("AdminTcp socket failed with error: %ld\n", WSAGetLastError());
			freeaddrinfo(result);
			WSACleanup();
			return;
		}

		// Setup the TCP listening socket
		iResult = bind(ListenSocket, result->ai_addr, (int)result->ai_addrlen);
		if (iResult == SOCKET_ERROR) {
			printf("AdminTcp bind failed with error: %d\n", WSAGetLastError());
			freeaddrinfo(result);
			closesocket(ListenSocket);
			WSACleanup();
			return;
		}

		freeaddrinfo(result);

		iResult = listen(ListenSocket, SOMAXCONN);
		if (iResult == SOCKET_ERROR) {
			printf("AdminTcp listen failed with error: %d\n", WSAGetLastError());
			closesocket(ListenSocket);
			WSACleanup();
			return;
		}

		// Accept a client socket
		ClientSocket = accept(ListenSocket, NULL, NULL);
		if (ClientSocket == INVALID_SOCKET) {
			printf("AdminTcp accept failed with error: %d\n", WSAGetLastError());
			closesocket(ListenSocket);
			WSACleanup();
			return;
		}
		printf("AdminTcp connected\n");
		// No longer need server socket
		closesocket(ListenSocket);

		// Receive until the peer shuts down the connection
		do {

			iResult = recv(ClientSocket, recvbuf, recvbuflen, 0);
			if (iResult > 0) {
				if (QueryMessage<MsgAdminGetNextCrumb>(recvbuf))
				{
					VectorInt32Math outCrumbPos;
					EtherColor outCrumbColor;
					bool bResult = ParallelPhysics::GetInstance()->GetNextCrumb(outCrumbPos, outCrumbColor);
					if (!bResult)
					{
						outCrumbPos = VectorInt32Math::ZeroVector;
						outCrumbColor = EtherColor::ZeroColor;
					}
					MsgAdminGetNextCrumbResponse msg;
					msg.m_color = outCrumbColor;
					msg.m_posX = outCrumbPos.m_posX;
					msg.m_posY = outCrumbPos.m_posY;
					msg.m_posZ = outCrumbPos.m_posZ;
					iSendResult = send(ClientSocket, msg.GetBuffer(), sizeof(msg), 0);
					if (iSendResult == SOCKET_ERROR) {
						printf("AdminTcp send failed with error: %d\n", WSAGetLastError());
						closesocket(ClientSocket);
						WSACleanup();
						return;
					}
				}
			}
			else if (iResult == 0)
			{
				printf("AdminTcp connection closed\n");
			}
			else
			{
				printf("AdminTcp recv failed with error: %d\n", WSAGetLastError());
				closesocket(ClientSocket);
				WSACleanup();
				return;
			}

		} while (iResult > 0);

		// shutdown the connection since we're done
		iResult = shutdown(ClientSocket, SD_SEND);
		if (iResult == SOCKET_ERROR) {
			printf("AdminTcp shutdown failed with error: %d\n", WSAGetLastError());
			closesocket(ClientSocket);
			WSACleanup();
			return;
		}

		// cleanup
		closesocket(ClientSocket);
		WSACleanup();
	}
}

}