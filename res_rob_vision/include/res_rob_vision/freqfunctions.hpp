#ifndef FREQUENT_FUNCTIONS_HPP
#define FREQUENT_FUNCTIONS_HPP

namespace FreqFunctions{

	static void appendPackagePath(std::string &filepath)
	{
	  std::string package_path(PACKAGE_PATH);
	  std::string return_path(package_path);
	  return_path.append(filepath);
	  filepath=return_path;
	}

	static cv::String appendPackagePath(const std::string filepath)
	{
	  std::string return_path(PACKAGE_PATH);
	  return_path.append(filepath);
	  return return_path;
	}

	template<typename T>
	void pop_front(std::vector<T>& vec)
	{
		assert(!vec.empty());
		vec.erase(vec.begin());
	}

	template<typename Ta>
	int vectorSequence(Ta &input_item, std::vector<Ta> &memory, int multitude)
	{
	    if(!input_item.empty())  {
	        // switch(true)  {
	        //     case memory.size()==0:
	        //         memory.push_back(input_item);
	        //         break;
	        //     case memory.size() < multitude:
	        //         memory.push_back(input_item);
	        //         break;
	        //     case memory.size() == multitude:
	        //         pop_front(memory);
	        //         memory.push_back(input_item);
	        //         break;
	        //     default:
	        //         memory.clear();
	        //         memory.push_back(input_item);
	        // }

	        if (memory.size()==0)  {
	            memory.push_back(input_item);
	        }  else if (memory.size() < multitude)  {
	            memory.push_back(input_item);
	        }  else if (memory.size() == multitude)  {
	            pop_front(memory);
	            memory.push_back(input_item);
	        }  else  {
	            memory.clear();
	            memory.push_back(input_item);
	        }
	    }
	    return memory.size();
	}


};

#endif // FREQUENT_FUNCTIONS_HPP