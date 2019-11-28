
/* 
 * File:   PhysicalSpacingTableData.h
 * Author: 
 *
 * Created on 12/2/2018, 17:35
 */

#ifndef PHYSICALDESIGN_PHYSICALSPACINGTABLEDATA_H
#define PHYSICALDESIGN_PHYSICALSPACINGTABLEDATA_H

namespace Rsyn {

class PhysicalSpacingTableData {
public:
	DBU clsParallelLength = 0;
	std::vector<DBU> clsParallelWidth;
	std::vector<DBU> clsParallelWidthSpacing;
	PhysicalSpacingTableData() = default;
}; // end class 

} // end namespace 

#endif /* PHYSICALDESIGN_PHYSICALSPACINGTABLEDATA_H */

