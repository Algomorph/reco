#!/usr/bin/python

import argparse
import sys
import xml.etree.ElementTree as ET
import os

FILTER_ELEMENT_NAME = "filter"
MATCHER_ELEMENT_NAME = "matcher"
ARGUMENTS_ELEMENT_NAME = "arguments"

def make_filter(filtered_resources_element,name,arguments,filter_type,filter_id):
	filter_elem = ET.SubElement(filtered_resources_element,FILTER_ELEMENT_NAME)
	id_elem = ET.SubElement(filter_elem,"id")
	id_elem.text = filter_id
	name_elem = ET.SubElement(filter_elem,"name")
	name_elem.text = name
	type_elem = ET.SubElement(filter_elem,"type")
	type_elem.text = filter_type
	matcher_elem = ET.SubElement(filter_elem,MATCHER_ELEMENT_NAME)
	m_id_elem = ET.SubElement(matcher_elem,"id")
	m_id_elem.text = "org.eclipse.ui.ide.multiFilter"
	args_elem = ET.SubElement(matcher_elem,ARGUMENTS_ELEMENT_NAME)
	args_elem.text = arguments

if __name__ == "__main__":
	conf_parser = argparse.ArgumentParser(description='Add exclude filter on [Subprojects] virtual folder in Eclipse project')
	conf_parser.add_argument('-p','--project_file', required=True, dest='proj_file')

	#parse the system arguments for the eclipse project file
	parsed = conf_parser.parse_args(sys.argv[1:])
	proj_file = parsed.proj_file

	#if project file is not found, do nothing
	if not os.path.isfile(proj_file):
		print "Project file not found: " + proj_file
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
	moc_excldr_filt = None
	
	subpr_text = "[Subprojects]"
	moc_excldr_args = "1.0-name-matches-false-false-moc_*"
	for f_elem in fr_elem.iter(FILTER_ELEMENT_NAME):
		nn = f_elem.find("name")
		if nn.text == subpr_text:
			subpr_filt = f_elem
		matcher_elem = f_elem.find(MATCHER_ELEMENT_NAME)
		args_elem = matcher_elem.find(ARGUMENTS_ELEMENT_NAME)
		if args_elem.text == moc_excldr_args:
			moc_excldr_filt = f_elem
	
	if(subpr_filt is None):
		#subprojects filter not found, create and add one
		make_filter(fr_elem, subpr_text,"1.0-name-matches-false-false-*", "30","1441290179575")
		#save the updated tree
		tree.write(proj_file)
	
	if(moc_excldr_filt is None):
		#subprojects filter not found, create and add one
		
		make_filter(fr_elem, "",moc_excldr_args, "22","1443117771675")
		#save the updated tree
		tree.write(proj_file)
