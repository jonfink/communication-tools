#include <vector>
#include "commsSimulatorBase.h"

#include <cstring>
#include <stdlib.h>
#include <stdio.h>

CommsSimulatorBase::CommsSimulatorBase(const map<int, agent_state_t> &current_state)
	: system_state(current_state)
{

}

CommsSimulatorBase::~CommsSimulatorBase()
{

}

void CommsSimulatorBase::AddParam(const string &key, const string &type, void *param_ptr)
{
  if(type == string("DOUBLE")) {
    param_types.insert(make_pair(key, type));    
    param_ptrs.insert(make_pair(key, (void*)param_ptr));
  }
  else if(type == string("INT")) {
    param_types.insert(make_pair(key, type));
    param_ptrs.insert(make_pair(key, param_ptr));
  }
  else if(type == string("DOUBLE_VECTOR")) {
    param_types.insert(make_pair(key, type));    
    param_ptrs.insert(make_pair(key, param_ptr));
  }
  else if(type == string("INT_VECTOR")) {
    param_types.insert(make_pair(key, type));    
    param_ptrs.insert(make_pair(key, param_ptr));
  }
}

void CommsSimulatorBase::SetParam(const string &key, const DOUBLE &value)
{
  if(param_types[key] == string("DOUBLE")) {
    if(param_ptrs[key] != NULL) {
      *reinterpret_cast<double*>(param_ptrs[key]) = value;
      return;
    }
  }

  fprintf(stderr, "Problem setting DOUBLE param %s\n", key.c_str());
}

void CommsSimulatorBase::SetParam(const string &key, const INT &value)
{
  if(param_types[key] == string("INT")) {
    if(param_ptrs[key] != NULL) {
      *reinterpret_cast<int*>(param_ptrs[key]) = value;
      return;
    }
  }
  
  fprintf(stderr, "Problem setting INT param %s\n", key.c_str());
}

void CommsSimulatorBase::SetParam(const string &key, const DOUBLE_VECTOR &value)
{
  if(param_types[key] == string("DOUBLE_VECTOR")) {
    if(param_ptrs[key] != NULL) {
      // Resize param vector if needed
      DOUBLE_VECTOR* param_vec = reinterpret_cast<DOUBLE_VECTOR*>(param_ptrs[key]);
      param_vec->resize(value.size());
      
      // copy elements into param vector
      for(int i = 0; i < value.size(); ++i) {
	(*param_vec)[i] = value[i];
      }
      return;
    }
  }
  
  fprintf(stderr, "Problem setting DOUBLE_VECTOR param %s\n", key.c_str());

}

void CommsSimulatorBase::SetParam(const string &key, const INT_VECTOR &value)
{
  if(param_types[key] == string("INT_VECTOR")) {
    if(param_ptrs[key] != NULL) {
      // Resize param vector if needed
      INT_VECTOR* param_vec = reinterpret_cast<INT_VECTOR*>(param_ptrs[key]);
      param_vec->resize(value.size());

      // copy elements into param vector
      for(int i = 0; i < value.size(); ++i) {
	(*param_vec)[i] = value[i];
      }
      return;
    }
  }
  
  fprintf(stderr, "Problem setting INT_VECTOR param %s\n", key.c_str());

}

void LabelMaterials(link_material_properties_t &prop)
{
  if(prop.segments_count <= 0) {
    fprintf(stderr, "No segments identified...\n");
    return;
  }

  if(prop.segment_materials != NULL)
    prop.segment_materials = (material_t*)realloc(prop.segment_materials, 
					      sizeof(double)*prop.segments_count);
  else
    prop.segment_materials = (material_t*)malloc(sizeof(double)*prop.segments_count);

  for(int i=0; i < prop.segments_count; ++i) {
    prop.segment_materials[i] = (material_t)(i % 2);
  }
}
