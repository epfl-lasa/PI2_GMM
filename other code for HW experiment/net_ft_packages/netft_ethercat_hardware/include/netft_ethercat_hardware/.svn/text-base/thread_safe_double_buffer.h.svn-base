/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef NETFT_ETHERCAT_HARDWARE__THREAD_SAFE_DOUBLE_BUFFER
#define NETFT_ETHERCAT_HARDWARE__THREAD_SAFE_DOUBLE_BUFFER

#include <boost/thread/mutex.hpp>

namespace netft_ethercat_hardware
{

template<class T>
class ThreadSafeDoubleBuffer
{
public:
  ThreadSafeDoubleBuffer();
  ThreadSafeDoubleBuffer(const T &initial_value);
  
  void put(const T &value);  //!<  Copies value into internal buffer.  Thread safe, may block
  void get(T &value);  //!<  Copies newest data from interval buffer to argument, Thread safe, unlikely to block.

protected:
  T v1_, v2_;
  boost::mutex mutex1_;
  boost::mutex mutex2_;
  unsigned blocked_counter_;
};


template<class T>  
ThreadSafeDoubleBuffer<T>::ThreadSafeDoubleBuffer() : blocked_counter_(0) 
{ 
  // empty
}

template<class T>  
ThreadSafeDoubleBuffer<T>::ThreadSafeDoubleBuffer(const T &initial_value) : 
  blocked_counter_(0), v1_(initial_value), v2_(initial_value)
{ 
  // empty
}

template<class T>
void ThreadSafeDoubleBuffer<T>::put(const T &value)
{
  // Copy of value into both buffers, only hold lock to one buffer at a time
  { boost::unique_lock<boost::mutex> lock(mutex1_);
    v1_ = value;
  }
  { boost::unique_lock<boost::mutex> lock(mutex2_);
    v2_ = value;
  }
}

template<class T>
void ThreadSafeDoubleBuffer<T>::get(T &value)
{
  // Try getting lock to one of the buffer's
  if (mutex1_.try_lock())
  {
    value = v1_;
    mutex1_.unlock();
    return;
  }
  if (mutex2_.try_lock())
  {
    value = v2_;
    mutex2_.unlock();
    return;
  }
  // OK, block waiting for buffer to open-up
  { boost::unique_lock<boost::mutex> lock(mutex1_);
    v1_ = value;
    ++blocked_counter_;
  }
}    

} //end namespace netft_ethercat_hardware


#endif // NETFT_ETHERCAT_HARDWARE__THREAD_SAFE_DOUBLE_BUFFER
