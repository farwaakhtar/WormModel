#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/random-variable-stream.h"
#include "ns3/olsr-module.h"
#include "ns3/aodv-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/traced-value.h"
#include "ns3/queue.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/internet-module.h"
#include "ns3/packet-sink.h"
#include "ns3/point-to-point-dumbbell.h"
#include "ns3/netanim-module.h"
#include "ns3/csma-module.h"
#include "ns3/ipv4-nix-vector-helper.h"

#include <string>
#include <fstream>
#include <sstream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

using namespace ns3;
using namespace std;

int main(int argc, char* argv[]) {
 
/***********************************************//***********************************************/
// Define some variables. 
/***********************************************//***********************************************/       
    uint32_t nClients = 0;
    uint16_t fanout = 4;		
    double addressUtil = 0.5;
    uint16_t depth = 4;
    double pChild = 0;

    CommandLine cmd;
    cmd.AddValue("fanout", "Number of nodes per parent", fanout);
    cmd.AddValue("nClients", "Number of client nodes in the simulation", nClients);
    cmd.AddValue("depth", "Depth of the random tree topology, including the client nodes", depth);
    cmd.AddValue("addressUtil", "Utilization of the addresses", addressUtil);
    cmd.Parse (argc, argv);
    
/***********************************************//***********************************************/
// Fix fanout and Depth Values and AddressUtil 
/***********************************************//***********************************************/  
    if (depth <2)
    {
	cout << "depth must be at least 2. depth of 4 has been automatically selected" << endl;
	depth = 4;
    }
    if (fanout < 2)
    {
	cout << "fanout must be at least 2. fanout of 4 has been automatically selected" << endl;
	fanout = 4;
    }
    if ( addressUtil < 0 || addressUtil > 1 )
    {
	cout << "addressUtil must be between 0 and 1. addressUtil of 0.5 has been automatically selected" << endl;
	addressUtil = 0.5;
    }
    
/***********************************************//***********************************************/
// Calculate pChild, and fanout (if nClients specified) OR nClients
/***********************************************//***********************************************/      
  pChild = exp ( (log(addressUtil))/(depth - 1) );  
  if ( nClients != 0 )
  {
	cout << "You have specified nClients. The value of fanout will be calculated again, overriding any value you may have entered for it" << endl;
	fanout = ceil ( double ( (exp ( log(nClients)/ (depth - 1) ) )/ pChild ) );
  }	
  else
  	nClients = pow(fanout * pChild, depth - 1); 
  uint16_t maxClientSubnets = pow(fanout, depth - 2);			//Maximum number of possible client/end host subnets

/***********************************************//***********************************************/
// Random Variable Declaration and Initialization
/***********************************************//***********************************************/
  RngSeedManager::SetSeed(11223344); 		 			
  Ptr<UniformRandomVariable> randVar;	
  randVar = CreateObject<UniformRandomVariable> ();
  randVar->SetAttribute ("Stream", IntegerValue (6110));
  randVar->SetAttribute ("Min", DoubleValue (0));
  randVar->SetAttribute ("Max", DoubleValue (1));
  
/***********************************************//***********************************************/
// Define internet stack, address space and link properties for routers
/***********************************************//***********************************************/
  InternetStackHelper internetStack;
  Ipv4AddressHelper addresses;
  Ipv4NixVectorHelper nixRouting;
  internetStack.SetRoutingHelper (nixRouting);
  addresses.SetBase ("10.0.0.0","255.255.255.252");

  PointToPointHelper p2pLinks;						//p2pLink is the link between all p2p devices i.e. routers
  p2pLinks.SetDeviceAttribute ("DataRate", StringValue ("100Mbps"));
  p2pLinks.SetChannelAttribute ("Delay", StringValue ("100ms"));

/***********************************************//***********************************************/
// Define some variables for locations on NetAnim (we want a star like topology)
/***********************************************//***********************************************/ 
  double angle = 2 * M_PI / fanout;					//how much angle between each child node of a particular parent node
  double angleOffset = 0;						//for the next tree/depth level, we want to have the nodes rotated. use offset
  double minDistance = 5;						//the minimum distance between parent and child. this will be for the last depth level
  double distance [depth-1];						//distance between parent and each of its children at a specific depth
  distance[depth-2] = minDistance;					//set the distance at the last depth level to be the minimum distance
  double center = 0; 							//center defines the position of the root node.
  for (int16_t i = (depth - 3) ; i >= 0; i--)				//starting from the second last depth going up, determine distance for each depth level
  	distance[i] = distance[i+1] * sqrt (depth);	
  for (int16_t i = 0; i < (depth - 1); i++)				//summing over all depths, find the distance from center to edge. This will be the co-ordinate of the root node
  {
	if (i%2 == 0)	
		center += distance[i];
	else
		center += distance[i] * cos(angle/2);		
  }
  
  Ptr<ListPositionAllocator> initialAlloc = CreateObject<ListPositionAllocator> ();	//create a vector for assigning node positions
  initialAlloc->Add (Vector ( center , center, 0));					//Add the position of the root

  MobilityHelper mobility;								//define the mobility. 
  mobility.SetPositionAllocator(initialAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

/***********************************************//***********************************************/
// Create the root node
/***********************************************//***********************************************/    
  NodeContainer allNodes;						//single dimension container for all nodes. Needed for position assignment for the NetAnim
  NodeContainer nodesAtDepth [depth];					//Contains all nodes such that the index refers to the depth of the tree and contains all nodes at that depth. Includes clients at the last depth level
  NodeContainer clientNodes [maxClientSubnets];				//contains all the nodes for each client subnet, indexed by the physical subnet number. Last element of this array contains the gateway router for that subnet
cout << "created nodes" << endl; 								
  nodesAtDepth[0].Create(1);						//create root node at the first depth level
  allNodes.Add(nodesAtDepth[0]);					//add it to the container for all nodes
  internetStack.Install(nodesAtDepth[0]);				//install stack on it
  mobility.Install(nodesAtDepth[0]);					//install position on it

/***********************************************//***********************************************/
// Create the router & clients, nodes determine their positions. Only routers IPs assigned here
/***********************************************//***********************************************/  
  Ipv4InterfaceContainer parentInterfaces [depth-1];			//these two contain the pair of IP addresses between each parent-child combination at a particular depth level
  Ipv4InterfaceContainer childInterfaces [depth-1];
  Ipv4InterfaceContainer clientInterfaces [maxClientSubnets];		//contains IP addresses of clients indexed by the physical subnet they belong to. First entry contains the local IP of the gateway router	
  NetDeviceContainer tempNet;						//a temporary net device container used for p2p link assignment
  Ipv4InterfaceContainer tempIpv4;					//a temporary IP interface container used for IP assignment for p2p device pairs
  
  bool parentExists [ uint16_t (pow(fanout, (depth-1))) ];		//this determines whether for a particular position at a depth level if the parent for that position exits
  bool amParent [ uint16_t (pow(fanout, (depth-1))) ];			//this tells us that at a particular depth level, node at this index has been created and can be parent for the next depth level
  for (uint16_t i = 0; i < pow(fanout, (depth-1) ); i++)		//initialize both of these to false
  {
	parentExists[i] = false;
	amParent[i] = false;
  }
  parentExists[0] = true;						//for the first level, parent exists so set it as such
  					
  uint16_t k = 0;							//for separating each node at a particular depth with respect to their physical parent node. k is the parent
  uint16_t l = 0; 							//for indexing nodes at each depth. Extra variable l is used because we skip some positions and need to keep track of the physically created nodes
  bool parentIterated = false;						//helps us determine when to increment k. Only increment k when, for a particular exisiting parent we have checked all possible child positions
  double x_base, y_base, x_pos, y_pos;					//base refers to co-ordinated of parent of a node. pos refers to its position
  
  for (uint16_t i = 1; i < depth; i++)						//for each depth level after the root node ( i = 0 here will refer to the depth level of the root)
  {
  	parentIterated = false;	
	k = 0;										
	l = 0;										
	for (uint16_t j = 0; j < pow(fanout,i); j++)				//for all possible node positions at the ith depth (does not mean nodes will exist at those positions!)
	{
		if (j%fanout == 0 && (parentExists[j/fanout]) == true )		//after every fanout amount of iterations, we will be talking about a different parent to the current node(s). Here we determine the parent
		{								//we only want to increment k once we go over fanout amount of iterations, but only if a parent existed for those corresponding iterations
			if (parentIterated)
				k++;
			else
				parentIterated = true;
		}
		else if (j%fanout == 0 && (parentExists[j/fanout]) == false && (parentIterated) )
		{
			k++;
			parentIterated = false;
		}
	
		if ( randVar->GetValue() < pChild && (parentExists[j/fanout]) )		//if we can probablistically create a node at this position and for that position a parent exists
		{			
			nodesAtDepth[i].Create(1);				//create node
			amParent[j] = true;					//this node can be a parent for the next depth level. Mark it as such. For the last depth level, this tells us that a client exists at that position
			
			if (i != (depth - 1))						//if depth is not the last we want to assign IP addresses to the nodes (which will be routers)
			{
				internetStack.Install(nodesAtDepth[i].Get(l));		//install internet stack on the created node (router)			
			
				tempNet = p2pLinks.Install (nodesAtDepth[i-1].Get(k), nodesAtDepth[i].Get(l));		//assign net devices and IP addresses to each interface of the link i.e the kth parent (lies at the depth i-1) 
															//and the lth child (at current depth i)
				tempIpv4 = addresses.Assign (NetDeviceContainer(tempNet.Get(0),tempNet.Get(1)));		
				parentInterfaces[i-1].Add (tempIpv4.Get(0));						//save IP address of parent
				childInterfaces[i-1].Add (tempIpv4.Get(1));						//save IP address of child	
				addresses.NewNetwork ();								//move to the next subnet
			}
			else												//if last level of depth we have created a client. We will do IP addressing later
				clientNodes[k].Add(nodesAtDepth[i].Get(l));						//here just add client to a separate node container indexed by the subnet/parent (k)
						
			x_base = (nodesAtDepth[i-1].Get(k)->GetObject<MobilityModel> ())->GetPosition().x;		//Get the x and y positions of the parent for the current node
			y_base = (nodesAtDepth[i-1].Get(k)->GetObject<MobilityModel> ())->GetPosition().y;
			x_pos = x_base + distance[i-1]*cos((j%fanout)*angle+angleOffset);				//Calculate the positions of the of the current node with respect to their parent's position
			y_pos = y_base + distance[i-1]*sin((j%fanout)*angle+angleOffset);
			initialAlloc->Add (Vector ( x_pos , y_pos, 0));					//save it to the vector
			
			l++;												//increment the index for the physical node
		}
		else							//if cannot probablistically assign a node or parent doesn't exist
		{	
			amParent[j] = false;				//this position cannot be a parent for the next level (since no node created here)
			if (i != (depth - 1))				//if depth is not the last depth level 
				addresses.NewNetwork ();		//skip the subnet (again we are not dealing with IP addresses of clients in this loop so only skip for routers)
		}
        }
	if (i != (depth - 1))						//if depth is not the last depth level
	{
		for (k = 0; k < pow(fanout, (depth-1)); k++)		//here we move the current nodes which exist and can be parents for the next level to the parentExists variable but we only do this for routers. For clients
			parentExists[k] = amParent[k];			//we need information for their parents' existence as well as whether they are physically present, so we don't overwrite the array for the last depth level
	}
        allNodes.Add(nodesAtDepth[i]);							//add all nodes at current depth to the allNodes container
	mobility.Install (allNodes);							//install/update positions
	if (i%2 != 0)									//change the rotation of nodes with respect to their parent for the next level of nodes
	  angleOffset = angle/2;
	else
	  angleOffset = 0; 
  }
 cout << "created router and clients" << endl;
/***********************************************//***********************************************/
// Assign IP addresses to and CSMA characteristics to client nodes.
/***********************************************//***********************************************/ 
  CsmaHelper csma;							//define the csma helper
  csma.SetChannelAttribute ("DataRate", StringValue ("10Mbps"));
  csma.SetChannelAttribute ("Delay", StringValue ("6560ns"));

  uint16_t subnetSize = pow (2, ceil ( double (log2(fanout+3)) ) );	//subnet size for LAN will be 3 more than the fanout, since we need addreses for gateway router, network address and broadcast address. Set to power of 2
  string subnetMask;							//compute the subnet mask in dotted decimal form. 
  stringstream sM; sM << ( 256 - subnetSize ); 
  subnetMask = "255.255.255." + sM.str();

  addresses.SetBase ("20.0.0.0", Ipv4Mask(subnetMask.c_str()));		//change the address space for the clients
  NetDeviceContainer clientDevices [nodesAtDepth[depth-2].GetN()]; 	//create a net device container for client nodes, indexed by their physical subnet number

  k = 0;									//keep track of the physical subnet number
  Ipv4Address ipv4Address;							//for storing the last skipped IP address (NO BEARING ON RESULT JUST FOR BULD APPLICATION)
  for (uint16_t i = 0; i < pow(fanout, (depth-2) ); i++)			//for all possible edge/gateway router positions
  {
	if ( parentExists[i] )							//if the edge router exists at the ith position
	{
		internetStack.Install(clientNodes[k]);				//install stack on client nodes
		clientNodes[k].Add(nodesAtDepth[depth-2].Get(k));		//add router to container of client nodes		
		clientDevices[k] = csma.Install(clientNodes[k]);		//install csma on all client nodes and the router at this physical subnet
		
		clientInterfaces[k].Add( addresses.Assign(clientDevices[k].Get( clientNodes[k].GetN()-1 )) );		//Assign IP address to the client facing interface of the edge/gateway router and save it
 		l = 0;								//keeps track of the physical client in this subnet
		for (uint16_t j = 0; j < fanout; j++)				//for each possible client/end host position
		{
			if (amParent[fanout*i+j])				//if client/host exists
			{
				clientInterfaces[k].Add(addresses.Assign(clientDevices[k].Get(l)));			//Assign it an IP address and save it
				l++;
			}
			else							//if at this position client/host node doesn't exist
				ipv4Address = addresses.NewAddress();		//skip this address
		}
		k++;								//increment the physical subnet number we are talking about
		addresses.NewNetwork();						//and move to the next subnet
	}
	else									//if the edge router does not exist at this position
		addresses.NewNetwork();						//skip this subnet and move to the next one
  }
  cout <<"aboveeeee" << endl;
  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();				//populate the global routing tables
	cout << "belowwwww" << endl;
/***********************************************//***********************************************/
// Make Animation
/***********************************************//***********************************************/  
  AnimationInterface animate("p4csma.xml");					//create the animation interface. Routers are blue and clients are green
  for (uint16_t i = 0; i < depth; i++)
  {	
	for (uint16_t j = 0; j < nodesAtDepth[i].GetN(); j++)				
	{
		if (i !=  (depth - 1) )
			animate.UpdateNodeColor(nodesAtDepth[i].Get(j)->GetId(), 0, 0, 255); 
		else
			animate.UpdateNodeColor(nodesAtDepth[i].Get(j)->GetId(), 0, 255, 0);
	}
  }

/***********************************************//***********************************************/
// PRINT IP's
/***********************************************//***********************************************/    
  // UNCOMMENT TO LIST IP ADDRESSES OF ROUTER INTERFACES 
  for (uint16_t i = 0; i < (depth - 2); i++)
  {
  	cout << "At depth " << i << " the parent-child IP address pairs (router-router) are: " << endl;		
	for (uint16_t j = 0; j < childInterfaces[i].GetN() ; j++) 
	{
		cout << parentInterfaces[i].GetAddress(j) << "<-->" << childInterfaces[i].GetAddress(j) << endl;
        }
	cout << endl;
  }//*/

  //UNCOMMENT TO PRINT ALL IPS OF CLIENTS
  k = 0;
  for (uint16_t i = 0; i < maxClientSubnets; i++)
  {	
	if ( parentExists[i] )
	{
		cout<< "Subnet#: " << i << " has " << (clientNodes[k].GetN() - 1) << " clients." << " The default gateway IP is: " <<  clientInterfaces[k].GetAddress(0) << endl;
		if (clientNodes[k].GetN() > 0)
		{
			cout << "The clients have the following IP addresses: " << endl;
			for (uint16_t j = 1; j < clientNodes[k].GetN(); j++)				
				cout << clientInterfaces[k].GetAddress(j) << endl;		
			cout<< endl;  
		}
		k++;
	}
	else
		cout<< "Subnet#: " << i << " does not exist." << endl << endl;
  }//*/

/***********************************************//***********************************************/
// Determine if nodes exist such that we can install Bulk send and Packet sink
/***********************************************//***********************************************/   
  ApplicationContainer sourceApps;
  ApplicationContainer sinkApps;
  ApplicationContainer sourceApps2;
  char continued = 0;
  NodeContainer tempNodes;
  for (uint16_t i = 0; i < k; i++)
  {
	if ( clientNodes[i].GetN() > 1 && (continued == 0) )							//create a bulk send a send packet to skipped IP address
	{	
		cout << clientInterfaces[i].GetAddress(1) << " sends packets to: " << ipv4Address << " which does NOT exist!" << endl;
		continued = 1;
		BulkSendHelper tcpSource2 ("ns3::TcpSocketFactory", InetSocketAddress (ipv4Address, 4000));
		tcpSource2.SetAttribute ("MaxBytes", UintegerValue (4000000000));
		tcpSource2.SetAttribute ("SendSize", UintegerValue (512));
		sourceApps2 = tcpSource2.Install (clientNodes[i].Get(0));
		sourceApps2.Start (Seconds (0));
		sourceApps2.Stop (Seconds (100));
		continue;
	}
	if ( clientNodes[i].GetN() > 1 && (continued == 1) )							//determine node for 2nd bulk app
	{	
		cout << clientInterfaces[i].GetAddress(1) << " sends packets to: ";
		tempNodes.Add(clientNodes[i].Get(0));
		continued = 2;
		continue;
	}
	if ( clientNodes[i].GetN() > 2 && (continued == 2) )							//create second bulk app and send packets to existing IP address
	{
		ipv4Address = clientInterfaces[i].GetAddress(2);
		cout << ipv4Address << endl;
		BulkSendHelper tcpSource ("ns3::TcpSocketFactory", InetSocketAddress (ipv4Address, 4000));
		tcpSource.SetAttribute ("MaxBytes", UintegerValue (4000000000));
		tcpSource.SetAttribute ("SendSize", UintegerValue (512));
		sourceApps = tcpSource.Install (tempNodes.Get(0));
		sourceApps.Start (Seconds (0));
		sourceApps.Stop (Seconds (100));

		PacketSinkHelper sink ("ns3::TcpSocketFactory", InetSocketAddress (Ipv4Address::GetAny(), 4000));
		sinkApps = sink.Install (clientNodes[i].Get(1));
		sinkApps.Start (Seconds (0));  
		sinkApps.Stop (Seconds (100));
		continued = 3;
		break;
	}
  }

  Simulator::Stop (Seconds (100));
  Simulator::Run ();
  Simulator::Destroy ();
  if (continued == 3)
  {
	Ptr<PacketSink> sink1 = DynamicCast<PacketSink> (sinkApps.Get (0));
	cout << ipv4Address <<  " receives BYTES: " << sink1->GetTotalRx () << endl;	
  }
/***********************************************//***********************************************/
/***********************************************//***********************************************/  
}

