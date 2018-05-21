#include <octomap/OcTreeBaseImpl.h>
#include <octomap/OcTree.h>
#include "octomap_ros/ColorOcTree.h"
#include <math.h>
#include <algorithm>
#include <octomap/AbstractOccupancyOcTree.h>

using namespace std;

namespace octomap {

  // node implementation
  std::ostream& ColorOcTreeNode::writeValue (std::ostream &s) const {
    // 1 bit for each children; 0: empty, 1: allocated
    std::bitset<8> children;
    for (unsigned int i=0; i<8; i++) {
      if (childExists(i)) children[i] = 1;
      else                children[i] = 0;
    }
    char children_char = (char) children.to_ulong();

    // write node data
    s.write((const char*) &value, sizeof(value)); // occupancy
    s.write((const char*) &color, sizeof(Color)); // color
  //  s.write((const char *)&id,sizeof(id)); //id
    s.write((char*)&children_char, sizeof(char)); // child existence

    // write existing children
    for (unsigned int i=0; i<8; ++i)
      if (children[i] == 1) this->getChild(i)->writeValue(s);
    return s;
  }

  std::istream& ColorOcTreeNode::readValue (std::istream &s) {
    // read node data
    char children_char;
    s.read((char*) &value, sizeof(value)); // occupancy
    s.read((char*) &color, sizeof(Color)); // color
  //  s.read((char *)&id,sizeof(id)); //id
    s.read((char*)&children_char, sizeof(char)); // child existence

    // read existing children
    std::bitset<8> children ((unsigned long long) children_char);
    for (unsigned int i=0; i<8; i++) {
      if (children[i] == 1){
        createChild(i);
        getChild(i)->readValue(s);
      }
    }
    return s;
  }

  ColorOcTreeNode::Color ColorOcTreeNode::getAverageChildColor() const {
    int mr(0), mg(0), mb(0);
    int c(0);
    for (int i=0; i<8; i++) {
      if (childExists(i) && getChild(i)->isColorSet()) {
        mr += getChild(i)->getColor().r;
        mg += getChild(i)->getColor().g;
        mb += getChild(i)->getColor().b;
        ++c;
      }
    }
    if (c) {
      mr /= c;
      mg /= c;
      mb /= c;
      return Color((unsigned char) mr, (unsigned char) mg, (unsigned char) mb);
    }
    else { // no child had a color other than white
      return Color(255, 255, 255);
    }
  }


  void ColorOcTreeNode::updateColorChildren() {
    color = getAverageChildColor();
  }

  // pruning-consider prune it when the id are the same
  bool ColorOcTreeNode::pruneNode() {
    // checks for equal occupancy only, color ignored
    if (!this->collapsible()) return false;
    // set occupancy value
    setLogOdds(getChild(0)->getLogOdds());
    //set id
    setId(getChild(0)->getId());
    // set color to average color
    if (isColorSet()) color = getAverageChildColor();
    // delete children
    for (unsigned int i=0;i<8;i++) {
      delete children[i];
    }
    delete[] children;
    children = NULL;
    return true;
  }

  void ColorOcTreeNode::expandNode() {
    assert(!hasChildren());
    for (unsigned int k=0; k<8; k++) {
      createChild(k);
      children[k]->setValue(value);
      getChild(k)->setColor(color);
      getChild(k)->setId(id);
    }
  }

  // tree implementation
  ColorOcTree::ColorOcTree(double resolution)
  : OccupancyOcTreeBase<ColorOcTreeNode>(resolution) {
    ColorOcTreeMemberInit.ensureLinking();
  };

  ColorOcTreeNode* ColorOcTree::setNodeColor(const OcTreeKey& key,
                                             const unsigned char& r,
                                             const unsigned char& g,
                                             const unsigned char& b) {
    ColorOcTreeNode* n = search (key);

    if (n != 0) {
      n->setColor(r, g, b);
    }
    return n;
  }

      ColorOcTreeNode* ColorOcTree::setNodeId(const OcTreeKey& key,
                                                  const std::vector<int>  id){
          ColorOcTreeNode* n = search (key);
          if (n != 0) {
            n->setId(id);
          }
          return n;
      }

  ColorOcTreeNode* ColorOcTree::averageNodeColor(const OcTreeKey& key,
                                                 const unsigned char& r,
                                                 const unsigned char& g,
                                                 const unsigned char& b) {
    ColorOcTreeNode* n = search (key);
    if (n != 0) {
      if (n->isColorSet()) {
        ColorOcTreeNode::Color prev_color = n->getColor();
        n->setColor((prev_color.r + r)/2, (prev_color.g + g)/2, (prev_color.b + b)/2);
      }
      else {
        n->setColor(r, g, b);
      }
    }
    return n;
  }

  ColorOcTreeNode* ColorOcTree::IntegrateNodeColor(const OcTreeKey& key,
                                                   const unsigned char& r,
                                                   const unsigned char& g,
                                                   const unsigned char& b) {
    ColorOcTreeNode* n = search (key);
    if (n != 0) {
      if (n->isColorSet()) {
        ColorOcTreeNode::Color prev_color = n->getColor();
        double node_prob = n->getOccupancy();
        unsigned char new_r = (unsigned char) ((double) prev_color.r * node_prob
                                               +  (double) r * (0.99-node_prob));
        unsigned char new_g = (unsigned char) ((double) prev_color.g * node_prob
                                               +  (double) g * (0.99-node_prob));
        unsigned char new_b = (unsigned char) ((double) prev_color.b * node_prob
                                               +  (double) b * (0.99-node_prob));
        n->setColor(new_r, new_g, new_b);
      }
      else {
        n->setColor(r, g, b);
      }
    }
    return n;
  }

  ColorOcTreeNode* ColorOcTree::integrateNodeId(const OcTreeKey& key,
                                                    const unsigned int & id){
      ColorOcTreeNode* n = search (key);
      if (n != 0) {
          std::vector<int> a;
          a.push_back(id);
          n->setId(a);
      }
      return n;
  }

  ///not efficient enough
list<ColorOcTreeNode*> ColorOcTree::searchId(int id){
    list<ColorOcTreeNode*> idList;
    int depth = this->getTreeDepth();
    ColorOcTree::tree_iterator t_it = this->begin_tree(depth);
    ColorOcTree::tree_iterator t_end = this->end_tree();
    std::vector<int> a;
    a.push_back(id);
    for( ; t_it != t_end; ++t_it){
        if((*t_it).getId() == a){
           // cout<<(*t_it).getId()<<"\t\n";
            ColorOcTreeNode* res =&(* t_it);
            idList.push_back(res);
        }
    }
    return idList;
}

//for testing--compare the time between id-set and without it
void ColorOcTree::deleteIdWIthoutSet2(int id){
  //  list<ColorOcTreeNode*> idList;
    int depth = this->getTreeDepth();
    ColorOcTree::tree_iterator t_it = this->begin_tree(depth);
    ColorOcTree::tree_iterator t_end = this->end_tree();
    std::vector<int> a;
    a.push_back(id);
    for( ; t_it != t_end; ++t_it){
        if((*t_it).getId() == a){
            //find it! delete it!
            (* t_it).setLogOdds(octomap::AbstractOccupancyOcTree::getProbMissLog());
        }
    }
}


bool isContained(vector<int> id,int key){
    int low=0;
    int high=id.size()-1;
    int mid=-1;
    while(low <=high){
        mid=(low+high)/2;
        if(id[mid]==key){
            return true;
        }else if(id[mid]>key){
            high=mid-1;
        }else{
            low=mid+1;
        }
    }
    return false;
}

///using id-structure to search then delete node(set unoccupied)
void ColorOcTree::deleteById(int id){
    this->searchDeleteById(this->root,0,id);
//    this->updateInnerOccupancy();
}

void ColorOcTree::searchDeleteById(ColorOcTreeNode* node, unsigned int depth,int sId/*,Project2Dmap & pMap*/){
    if(isContained(node->id,sId)){
        if (node->hasChildren()){
          if (depth < this->tree_depth)
//              if(isContained(node->id,sId))
                  for (unsigned int i=0; i<8; i++)
                    if (node->childExists(i))
                      searchDeleteById(node->getChild(i), depth+1,sId);
        }else{
             if(isContained(node->id,sId)){
                 //set unoccupied
                 node->setLogOdds(octomap::AbstractOccupancyOcTree::getProbMissLog());
                 node->setUpdatePro = true;//update project
             }
             //father node
             else return;
        }
    }else{
        return;
    }
}

///without id-set
void ColorOcTree::deleteByIdWithoutSet(int id){
      this->searchDeleteByIdWithoutSet(this->root,0,id);
}

void ColorOcTree::searchDeleteByIdWithoutSet(ColorOcTreeNode* node, unsigned int depth,int sId){
    if (node->hasChildren()){
      if (depth < this->tree_depth)
         // if(isContained(node->id,sId))
              for (unsigned int i=0; i<8; i++)
                if (node->childExists(i))
                  searchDeleteById(node->getChild(i), depth+1,sId);
    }else{ //leaf
         if(isContained(node->id,sId))
             //set unoccupied
             node->setLogOdds(octomap::AbstractOccupancyOcTree::getProbMissLog());
    }
}



//using id-structure to search
//for test
list<ColorOcTreeNode*> ColorOcTree::searchId2(int id){
        list<ColorOcTreeNode*> idList;
        this->searchDeleteById(idList,this->root,0,id);
        return idList;
}
//for test
void ColorOcTree::searchDeleteById( list<ColorOcTreeNode*> & idList ,ColorOcTreeNode* node, unsigned int depth,int sId){
    if (node->hasChildren()){
      if (depth < this->tree_depth)
          if(isContained(node->id,sId))
              for (unsigned int i=0; i<8; i++)
                if (node->childExists(i))
                  searchDeleteById(idList,node->getChild(i), depth+1,sId);
    }else{
         if(isContained(node->id,sId))
             idList.push_back(node);
    }
}

///print tree-to see structure
 void ColorOcTree::printTree(ColorOcTreeNode * node, unsigned int depth){
     if (node->hasChildren()){
         cout<<"------------------"<<depth<<" depth hasChildren enter.\n";
       if (depth < this->tree_depth){
         for (unsigned int i=0; i<8; i++) {
           if (node->childExists(i)) {
                cout<<i<<" children exist enter.recursive\n";
               printTree(node->getChild(i), depth+1);
           }else{
               cout<<i<<" children not exist\n";
           }
         }
       }
     }else{
         cout<<"depth:"<<depth<<",id:"<<node->id[0]<<endl;
     }
 }

  void ColorOcTree::updateInnerOccupancy() {
    this->updateInnerOccupancyRecurs(this->root, 0);
  }

  //prune tree
  void ColorOcTree::pruneTree(ColorOcTreeNode* node, unsigned int depth){
      if (node->hasChildren()){
        if (depth < this->tree_depth){
            if(node->childExists(0) && node->childExists(1) && node->childExists(2) && node->childExists(3) &&
                node->childExists(4) && node->childExists(5) &&node->childExists(6) && node->childExists(7)){
                //if they are all leaves && their ids are the same, then prune it
                if(!node->getChild(0)->hasChildren()&&!node->getChild(1)->hasChildren()&&!node->getChild(2)->hasChildren()&&!node->getChild(3)->hasChildren()&&
                   !node->getChild(4)->hasChildren()&&!node->getChild(5)->hasChildren()&&!node->getChild(6)->hasChildren()&&!node->getChild(7)->hasChildren()){
                    if( (node->getChild(0)->id ==node->getChild(1)->id) &&(node->getChild(1)->id ==node->getChild(2)->id)  &&
                        (node->getChild(2)->id ==node->getChild(3)->id)  && (node->getChild(3)->id ==node->getChild(4)->id) &&
                        (node->getChild(4)->id ==node->getChild(5)->id) && (node->getChild(5)->id ==node->getChild(6)->id)  &&
                        (node->getChild(6)->id ==node->getChild(7)->id))
                        node->pruneNode();
                }
            }else{
                for (unsigned int i=0; i<8; i++) {
                  if (node->childExists(i)) {
                    pruneTree(node->getChild(i), depth+1);
                  }
                }
            }
        }
      }
  }

  void ColorOcTree::updateInnerOccupancyRecurs(ColorOcTreeNode* node, unsigned int depth) {
    // only recurse and update for inner nodes
    if (node->hasChildren()){
      // return early for last level:
      if (depth < this->tree_depth){
        for (unsigned int i=0; i<8; i++) {
          if (node->childExists(i)) {
            updateInnerOccupancyRecurs(node->getChild(i), depth+1);
            //integrate id into father node
            for(size_t k =0;k<node->getChild(i)->id.size();k++)
                node->id.push_back(node->getChild(i)->id[k]);
            //delete duplication
            sort(node->id.begin(),node->id.end());
            node->id.erase(unique(node->id.begin(), node->id.end()), node->id.end());
          }
        }
      }
      node->updateOccupancyChildren();
      node->updateColorChildren();
    }
  }

  void ColorOcTree::writeColorHistogram(std::string filename) {

#ifdef _MSC_VER
    fprintf(stderr, "The color histogram uses gnuplot, this is not supported under windows.\n");
#else
    // build RGB histogram
    std::vector<int> histogram_r (256,0);
    std::vector<int> histogram_g (256,0);
    std::vector<int> histogram_b (256,0);
    for(ColorOcTree::tree_iterator it = this->begin_tree(),
          end=this->end_tree(); it!= end; ++it) {
      if (!it.isLeaf() || !this->isNodeOccupied(*it)) continue;
      ColorOcTreeNode::Color& c = it->getColor();
      ++histogram_r[c.r];
      ++histogram_g[c.g];
      ++histogram_b[c.b];
    }
    // plot data
    FILE *gui = popen("gnuplot ", "w");
    fprintf(gui, "set term postscript eps enhanced color\n");
    fprintf(gui, "set output \"%s\"\n", filename.c_str());
    fprintf(gui, "plot [-1:256] ");
    fprintf(gui,"'-' w filledcurve lt 1 lc 1 tit \"r\",");
    fprintf(gui, "'-' w filledcurve lt 1 lc 2 tit \"g\",");
    fprintf(gui, "'-' w filledcurve lt 1 lc 3 tit \"b\",");
    fprintf(gui, "'-' w l lt 1 lc 1 tit \"\",");
    fprintf(gui, "'-' w l lt 1 lc 2 tit \"\",");
    fprintf(gui, "'-' w l lt 1 lc 3 tit \"\"\n");

    for (int i=0; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_r[i]);
    fprintf(gui,"0 0\n"); fprintf(gui, "e\n");
    for (int i=0; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_g[i]);
    fprintf(gui,"0 0\n"); fprintf(gui, "e\n");
    for (int i=0; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_b[i]);
    fprintf(gui,"0 0\n"); fprintf(gui, "e\n");
    for (int i=0; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_r[i]);
    fprintf(gui, "e\n");
    for (int i=0; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_g[i]);
    fprintf(gui, "e\n");
    for (int i=0; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_b[i]);
    fprintf(gui, "e\n");
    fflush(gui);
#endif
  }

  std::ostream& operator<<(std::ostream& out, ColorOcTreeNode::Color const& c) {
    return out << '(' << (unsigned int)c.r << ' ' << (unsigned int)c.g << ' ' << (unsigned int)c.b << ')';
  }


  ColorOcTree::StaticMemberInitializer ColorOcTree::ColorOcTreeMemberInit; 

  void ColorOcTree::projector2D(Project2Dmap & pmap){
      for( ColorOcTree::iterator it = this->begin(), end = this->end(); it != end; ++it )
      {
          double size = it.getSize(); //0.05
          double x = it.getX();
          double y = it.getY();
          double z = it.getZ();
          if(z>=pmap.zmin && z<=pmap.zmax){
              if(this->isNodeOccupied(*it) && !it->has_project && !it->hasChildren()){
                  //project
                  string Mo;//a0,b0;a1,b1
                  int a0,b0,a1,b1;
                  octomap::point3d nodeMin(x-0.5*size,y-0.5*size,0);
                  octomap::point3d nodeMax(x+0.5*size,y+0.5*size,0);
                  pmap.XY2ab(nodeMin,a0,b0);
                  pmap.XY2ab(nodeMax,a1,b1);
//                  cout<<"a0,b0 "<<a0<<","<<b0<<endl;cout<<"a1,b1 "<<a1<<","<<b1<<endl;
                  if(a0>a1){
                      a0 = a0+a1;
                      a1 = a0-a1;
                      a0 = a0-a1;
                  }
                  if(b0>b1){
                      b0 = b0+b1;
                      b1 = b0-b1;
                      b0 = b0-b1;
                  }
                  for(int i=a0;i<=a1;i++){
                      if(i==0)
                          i++;
                      for(int j=b0;j<=b1;j++){
                          if(j==0)
                              j++;
                          pmap.ab2Morton(i,j,Mo);
//                          cout<<"i,j,morton "<<i<<","<<j<<","<<Mo<<endl;
                          int count = pmap.map_grid.count(Mo);
                          if(count!=0){
                              ((pmap.map_grid.find(Mo))->second)->occupied +=1;
                          }else{
                              grid2D * g = new grid2D();
                              g->occupied += 1;
                              g->a = i;g->b=j;
                              pmap.map_grid.insert(pair<string,grid2D *>(Mo,g));
//                              cout<<"create-a,b"<<g->a<<","<<g->b<<endl;
                          }
//                          cout<<"add+occupied "<<((pmap.map_grid.find(Mo))->second)->occupied<<endl;
                      }
                  }
                  it->has_project = true;
              }else if(it->hasChildren()){
                  cout<<"projector2D error\n";
              }
          }
      }
  }

  //local update
  void ColorOcTree::updateProjectorMap(Project2Dmap & pmap){
      for( ColorOcTree::iterator it = this->begin(), end = this->end(); it != end; ++it )
      {
          double size = it.getSize(); //0.05
          double x = it.getX();
          double y = it.getY();
          double z = it.getZ();
          if(z>=pmap.zmin && z<=pmap.zmax){
              if( it->setUpdatePro == true && !it->hasChildren() && !this->isNodeOccupied(*it)){
                  //project
                  string Mo;//a0,b0;a1,b1
                  int a0,b0,a1,b1;
                  octomap::point3d nodeMin(x-0.5*size,y-0.5*size,0);
                  octomap::point3d nodeMax(x+0.5*size,y+0.5*size,0);
                  pmap.XY2ab(nodeMin,a0,b0);
                  pmap.XY2ab(nodeMax,a1,b1);
                  if(a0>a1){
                      a0 = a0+a1;
                      a1 = a0-a1;
                      a0 = a0-a1;
                  }
                  if(b0>b1){
                      b0 = b0+b1;
                      b1 = b0-b1;
                      b0 = b0-b1;
                  }
                  for(int i=a0;i<=a1;i++){
                      if(i==0)
                          i++;
                      for(int j=b0;j<=b1;j++){
                          if(j==0)
                              j++;
                          pmap.ab2Morton(i,j,Mo);
                          if(pmap.map_grid.count(Mo) !=0){
                              int occ = ((pmap.map_grid.find(Mo))->second)->occupied;
                              occ -=1;
                              ((pmap.map_grid.find(Mo))->second)->occupied = occ<0?0:occ;
                          }//else
//                              cout<<Mo<<"-delete update 2dmap error\n";//caused by pruneTree
//                          cout<<"occupied "<<((pmap.map_grid.find(Mo))->second)->occupied<<endl;
                      }
                  }
                  it->setUpdatePro = false;
                  it->has_project = false;
              }
          }
      }

  }

} // end namespace

