#include "slam_operations.h"
#include "dirent.h"

namespace slam_utility {

bool SlamOperation::GetFilesNameInDirectory(const std::string &dir, std::vector<std::string> &filenames) {
    DIR *ptr_dir;
    struct dirent *ptr;
    if (!(ptr_dir = opendir(dir.c_str()))) {
        return false;
    }

    while ((ptr = readdir(ptr_dir)) != 0) {
        CONTINUE_IF(strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0);
        CONTINUE_IF(dir.empty());
        if (dir.back() != '/') {
            filenames.emplace_back(dir + "/" + ptr->d_name);
        } else {
            filenames.emplace_back(dir + ptr->d_name);
        }
    }

    closedir(ptr_dir);
    return true;
}

}  // namespace slam_utility
