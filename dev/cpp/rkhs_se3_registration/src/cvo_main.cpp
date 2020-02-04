#include "cvo.hpp"

void load_file_name(string assoc_pth, vector<string> &vstrRGBName, \
                    vector<string> &vstrRGBPth, vector<string> &vstrDepPth);

void load_img(cv::Mat& RGB_img, cv::Mat& dep_img, string& RGB_pth, string& dep_pth);
void load_semantic_bin(const string label_bin_path, MatrixXf_row& semantic_labels, int rows, int cols, int num_class);
void load_mask(const string mask_path, bool* mask, int rows, int cols);


int main(int argc, char** argv){
    
    string folder = argv[1];
    int dataset_seq = std::stoi(argv[2]);
    string output_file = argv[3];
    string semantic_folder = argv[4];
    string mask_folder = argv[5];
    float s_ell = std::stof(argv[6]);
    float s_sigma = std::stof(argv[7]);

    // downsampled pcd from tum rgbd dataset
    string assoc_pth = folder + "assoc.txt";
    
    // create our registration class
    cvo::cvo cvo(s_ell, s_sigma);

    // load associate file
    vector<string> vstrRGBName;     // vector for image names
    vector<string> vstrRGBPth;
    vector<string> vstrDepPth;
    load_file_name(assoc_pth,vstrRGBName,vstrRGBPth,vstrDepPth);
    int num_img = vstrRGBName.size();
    std::cout<<"num images: "<<num_img<<std::endl;
    // num_img = 100;

    // // export as quarternion
    ofstream fPoseQtTxt;
    fPoseQtTxt.open(folder+output_file);

    boost::timer::cpu_timer total_time;
    cv::Mat RGB_img;
    cv::Mat dep_img;
    MatrixXf_row semantic_labels;
    bool* mask_map;

    // loop through all the images in associate files
    for(int i=0;i<num_img;i++){

        string RGB_pth = folder + vstrRGBPth[i];
        string dep_pth = folder + vstrDepPth[i];
        string semantic_path = folder + semantic_folder + vstrRGBName[i] + ".bin";
        // string semantic_path = folder + semantic_folder + "feat_" + vstrRGBName[i] + ".bin";
        // string mask_path = folder + semantic_folder + mask_folder + vstrRGBName[i] + ".bin";
        string pcd_load_pth = folder +"cvo_points/" + vstrRGBName[i] +".txt";
        string pcd_save_pth = folder +"pcd/"+ vstrRGBName[i] +".pcd";
        string pcd_dso_save_pth = folder +"pcd_dso/"+ vstrRGBName[i] + ".pcd";
        // std::cout<<"pcd pth: "<<pcd_save_pth<<std::endl;
        // std::cout<<"pcd dso pth: "<<pcd_dso_save_pth<<std::endl;
        
        if(cvo.init){
            std::cout<<"----------------------"<<std::endl;
            std::cout<<"Processing frame "<<i<<std::endl;
            std::cout<<"Aligning " + vstrRGBName[i-1] + " and " + vstrRGBName[i] <<std::endl;
        }

        boost::timer::cpu_timer timer;
        
        load_img(RGB_img, dep_img, RGB_pth, dep_pth);
        mask_map = new bool[RGB_img.rows*RGB_img.cols];
        load_semantic_bin(semantic_path, semantic_labels, RGB_img.rows, RGB_img.cols, NUM_CLASS);
        // load_mask(mask_path,mask_map, RGB_img.rows, RGB_img.cols);
        cvo.run_cvo(dataset_seq, RGB_img, dep_img, semantic_labels, mask_map, pcd_load_pth, pcd_dso_save_pth);
            
        std::cout<<"elapse time: "<<timer.format()<<std::endl;
        

        // log out quaternion
        if(cvo.init){

        // log out transformation matrix for each frame
            Eigen::Quaternionf q(cvo.accum_transform.matrix().block<3,3>(0,0));
            fPoseQtTxt<<vstrRGBName[i]<<" ";
            fPoseQtTxt<<cvo.accum_transform(0,3)<<" "<<cvo.accum_transform(1,3)<<" "<<cvo.accum_transform(2,3)<<" "; 
            fPoseQtTxt<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<"\n";
            fPoseQtTxt.flush();
        }

        delete mask_map;
    }

    std::cout<<"======================="<<std::endl;
    std::cout<<"Total time for "<<num_img<<" frames is: "<<total_time.format()<<std::endl;
    std::cout<<"======================="<<std::endl;

    fPoseQtTxt.close();
}

void load_file_name(string assoc_pth, vector<string> &vstrRGBName, \
                    vector<string> &vstrRGBPth, vector<string> &vstrDepPth){
    std::ifstream fAssociation;
    fAssociation.open(assoc_pth.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            string RGB;
            ss >> RGB;
            vstrRGBName.push_back(RGB);
            string RGB_pth;
            ss >> RGB_pth;
            vstrRGBPth.push_back(RGB_pth);
            string dep;
            ss >> dep;
            string depPth;
            ss >> depPth;
            vstrDepPth.push_back(depPth);
        }
    }
    fAssociation.close();
}


void load_img(cv::Mat& RGB_img, cv::Mat& dep_img, string& RGB_pth, string& dep_pth){
    RGB_img = cv::imread(RGB_pth);
    dep_img = cv::imread(dep_pth,CV_LOAD_IMAGE_ANYDEPTH);
}


void load_semantic_bin(const string label_bin_path, MatrixXf_row& semantic_labels, int rows, int cols, int num_class){

    semantic_labels = MatrixXf_row::Zero(rows*cols,NUM_CLASS);

    std::ifstream fLables(label_bin_path.c_str(),std::ios::in|std::ios::binary);
    if (fLables.is_open()){

        int mat_byte_size=sizeof(float)*rows*cols*NUM_CLASS; // byte number, make sure size is correct, or can use tellg to get the file size
        float *mat_field = semantic_labels.data();
        
        fLables.read((char*)mat_field,mat_byte_size);	
        fLables.close(); 
    }

    // make sure we got it ^__^
    // std::cout<<"label_bin_path "<<label_bin_path<<"\n";
    // std::cout<<"number of classes: "<<NUM_CLASS<<std::endl;
    // for(int r=0; r<30; ++r){
    //     for(int i=0; i<NUM_CLASS; ++i){
    //         std::cout<<semantic_labels(r,i)<<",";
    //     }
    //     std::cout<<std::endl;
    // }
}

void load_mask(const string mask_path, bool* mask, int rows, int cols){
    std::ifstream fMask(mask_path.c_str(),std::ios::in|std::ios::binary);
    if (fMask.is_open()){

        int mat_byte_size=sizeof(bool)*rows*cols*1; // byte number, make sure size is correct, or can use tellg to get the file size
        
        fMask.read((char*)mask,mat_byte_size);	
        fMask.close(); 
    }
}
