#include "Track.hpp"

Track::Track()
{

}
void Track::printTrack()
{
  print("GT: " << groundTruth.transpose());
  print("Color: " << color.transpose());
  print("number of points: " << occurrences.size());
  print("worldPosition:" << worldPosition.transpose());
  for(auto el: occurrences)
  {
    //print("camkey:" << el.first << ". Position:" << el.second(0) << "," <<el.second(1));
  }
}
