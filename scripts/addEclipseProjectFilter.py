#!/usr/bin/python

import argparse
import sys
import xml.etree.ElementTree as ET
import os

if __name__ == "__main__":
	parser = argparse.ArgumentParser(description='Add exclude filter on [Subprojects] virtual folder in Eclipse project')
	parser.add_argument('-p','--project_file', required=True, dest='proj_file')

	#parse the system arguments for the eclipse project file
	parsed = parser.parse_args(sys.argv[1:])
	proj_file = parsed.proj_file

	#if project file is not found, do nothing
	if not os.path.isfile(proj_file):
		sys.exit(1)

	#parse the eclipse project file as XML
	tree = ET.parse(proj_file)
	root = tree.getroot()
	#find the filteredResources element if one exists
	filt_rec_elem_name = "filteredResources"
	fr_elem = root.find(filt_rec_elem_name)
	if fr_elem is None:
		fr_elem = ET.SubElement(root,filt_rec_elem_name)
	#find appropriate filter if one is present
	subpr_filt = None
	filt_elem_name = "filter"
	subpr_text = "[Subprojects]"
	for f_elem in fr_elem.iter(filt_elem_name):
		nn = f_elem.findall("name")
		if len(nn) > 0 and nn[0].text == subpr_text:
			subpr_filt = f_elem
	#filter not found, create and add one
	#		<filter>
	#			<id>1441290179575</id>
	#			<name>[Subprojects]</name>
	#			<type>30</type>
	#			<matcher>
	#				<id>org.eclipse.ui.ide.multiFilter</id>
	#				<arguments>1.0-name-matches-false-false-*</arguments>
	#			</matcher>
	#		</filter>
	if(subpr_filt is None):
		subpr_filt = ET.SubElement(fr_elem,filt_elem_name)
		id_elem = ET.SubElement(subpr_filt,"id")
		id_elem.text = "1441290179575"
		name_elem = ET.SubElement(subpr_filt,"name")
		name_elem.text = subpr_text
		type_elem = ET.SubElement(subpr_filt,"type")
		type_elem.text = "30"
		matcher_elem = ET.SubElement(subpr_filt,"matcher")
		m_id_elem = ET.SubElement(matcher_elem,"id")
		m_id_elem.text = "org.eclipse.ui.ide.multiFilter"
		args_elem = ET.SubElement(matcher_elem,"arguments")
		args_elem.text = "1.0-name-matches-false-false-*"
		#save the updated tree
		tree.write(proj_file)
