// Passive ABI interposition in NEW owned diagnostic processes only.
// Calls exact installed functions with original arguments/results. No ROS/control output.
#include <google/protobuf/message.h>
#include <gz/msgs/image.pb.h>
#include <chrono>
#include <dlfcn.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <time.h>
#include <cstdio>
#include <cstdlib>
#include <cerrno>
#include <atomic>
#include <execinfo.h>
#include <cstring>
#include <link.h>
static thread_local long long active_source=-1;
static long long ns(clockid_t c){timespec t;clock_gettime(c,&t);return (long long)t.tv_sec*1000000000+t.tv_nsec;}
static std::atomic<int> logfd{-2};
static void record(char kind,long long sim,long long start,long long end,long long cpu,int result){
 int fd=logfd.load();if(fd==-2){const char*p=getenv("CAMERA_TRACE_PREFIX");if(!p){logfd=-1;return;}char path[256];snprintf(path,sizeof(path),"%s_%d.csv",p,(int)getpid());int f=open(path,O_WRONLY|O_CREAT|O_EXCL|O_APPEND,0644);int expected=-2;if(!logfd.compare_exchange_strong(expected,f)&&f>=0)close(f);fd=logfd.load();}if(fd<0)return;
 char row[256];int n=snprintf(row,sizeof(row),"%c,%lld,%lld,%lld,%lld,%ld,%d,%d\n",kind,sim,start,end,cpu,syscall(SYS_gettid),(int)getpid(),result);int saved=errno;auto written=write(fd,row,n);(void)written;errno=saved;
}
struct Lookup {const char*symbol;void*result;};
static bool own(void*p){Dl_info i{};return !p||!dladdr(p,&i)||!i.dli_fname||strstr(i.dli_fname,"native_parts");}
static int lookup_loaded(dl_phdr_info*i,size_t,void*opaque){auto*q=(Lookup*)opaque;if(!i->dlpi_name||!i->dlpi_name[0])return 0;void*h=dlopen(i->dlpi_name,RTLD_NOW|RTLD_NOLOAD);void*p=h?dlsym(h,q->symbol):nullptr;if(!own(p)){q->result=p;return 1;}return 0;}
static void *actual(const char*symbol,const char*library){
 void*h=dlopen(library,RTLD_NOW|RTLD_NOLOAD);void*p=h?dlsym(h,symbol):nullptr;
 if(own(p))p=dlsym(RTLD_NEXT,symbol);
 if(own(p)){Lookup q{symbol,nullptr};dl_iterate_phdr(lookup_loaded,&q);p=q.result;}
 if(own(p)){fprintf(stderr,"native parts cannot resolve actual %s\n",symbol);abort();}return p;
}

extern "C" bool sensor_update(void*,const std::chrono::nanoseconds&) asm("_ZN2gz7sensors2v812CameraSensor6UpdateERKNSt6chrono8durationIlSt5ratioILl1ELl1000000000EEEE");
extern "C" bool sensor_update(void*self,const std::chrono::nanoseconds&t){
 using F=bool(*)(void*,const std::chrono::nanoseconds&);static auto f=(F)actual("_ZN2gz7sensors2v812CameraSensor6UpdateERKNSt6chrono8durationIlSt5ratioILl1ELl1000000000EEEE","/opt/ros/jazzy/opt/gz_sensors_vendor/lib/libgz-sensors8-camera.so");auto s=ns(CLOCK_MONOTONIC),c=ns(CLOCK_THREAD_CPUTIME_ID);long long old=active_source;active_source=t.count();bool ok=f(self,t);record('S',t.count(),s,ns(CLOCK_MONOTONIC),ns(CLOCK_THREAD_CPUTIME_ID)-c,ok);active_source=old;return ok;
}
extern "C" bool image_publish(void*,const google::protobuf::Message&) asm("_ZN2gz9transport3v134Node9Publisher7PublishERKN6google8protobuf7MessageE");
extern "C" bool image_publish(void*self,const google::protobuf::Message&m){
 using F=bool(*)(void*,const google::protobuf::Message&);static auto f=(F)actual("_ZN2gz9transport3v134Node9Publisher7PublishERKN6google8protobuf7MessageE","/opt/ros/jazzy/opt/gz_transport_vendor/lib/libgz-transport13.so");if(m.GetTypeName()!="gz.msgs.Image")return f(self,m);
 auto &im=static_cast<const gz::msgs::Image&>(m);long long t=(long long)im.header().stamp().sec()*1000000000+im.header().stamp().nsec();auto s=ns(CLOCK_MONOTONIC),c=ns(CLOCK_THREAD_CPUTIME_ID);bool ok=f(self,m);record('P',t,s,ns(CLOCK_MONOTONIC),ns(CLOCK_THREAD_CPUTIME_ID)-c,ok);return ok;
}
static thread_local const void *last_image=nullptr;
static thread_local long long last_stamp=0;
extern "C" void convert_image(const gz::msgs::Image&,void*) asm("_ZN13ros_gz_bridge17convert_gz_to_rosIN11sensor_msgs3msg6Image_ISaIvEEEN2gz4msgs5ImageEEEvRKT0_RT_");
extern "C" void convert_image(const gz::msgs::Image&m,void*out){
 using F=void(*)(const gz::msgs::Image&,void*);static auto f=(F)actual("_ZN13ros_gz_bridge17convert_gz_to_rosIN11sensor_msgs3msg6Image_ISaIvEEEN2gz4msgs5ImageEEEvRKT0_RT_","/opt/ros/jazzy/lib/libros_gz_bridge.so");auto s=ns(CLOCK_MONOTONIC),c=ns(CLOCK_THREAD_CPUTIME_ID);f(m,out);long long t=(long long)m.header().stamp().sec()*1000000000+m.header().stamp().nsec();last_image=out;last_stamp=t;record('B',t,s,ns(CLOCK_MONOTONIC),ns(CLOCK_THREAD_CPUTIME_ID)-c,1);
}
// Associate bridge conversion output pointer with its actual rcl publish call.
// The conversion wrapper sets these only for real Image messages.

extern "C" int rcl_publish(const void*pub,const void*msg,void*allocation){
 using F=int(*)(const void*,const void*,void*);static auto f=(F)actual("rcl_publish","/opt/ros/jazzy/lib/librcl.so");
 if(msg!=last_image)return f(pub,msg,allocation);
 auto s=ns(CLOCK_MONOTONIC),c=ns(CLOCK_THREAD_CPUTIME_ID);int ok=f(pub,msg,allocation);record('R',last_stamp,s,ns(CLOCK_MONOTONIC),ns(CLOCK_THREAD_CPUTIME_ID)-c,ok);return ok;
}

// Boundary call chains, not statistical CPU stack samples. No signals/ptrace.
static void boundary_stack(char kind){
 static std::atomic<unsigned> seen[256];if(seen[(unsigned char)kind].fetch_add(1)!=0)return;
 void*frames[32];int n=backtrace(frames,32);char path[128];snprintf(path,sizeof(path),"/tmp/phase1_native_stack_%d.csv",(int)getpid());int fd=open(path,O_WRONLY|O_CREAT|O_APPEND,0644);if(fd<0)return;
 for(int j=0;j<n;j++){Dl_info info{};dladdr(frames[j],&info);char line[1024];int z=snprintf(line,sizeof(line),"%c,%lld,%ld,%d,%s,%s,%llx\n",kind,ns(CLOCK_MONOTONIC),syscall(SYS_gettid),j,info.dli_fname?info.dli_fname:"?",info.dli_sname?info.dli_sname:"?",(unsigned long long)((char*)frames[j]-(char*)info.dli_fbase));auto written=write(fd,line,z);(void)written;}
 close(fd);
}

extern "C" void hook_J(void*) asm("_ZN2gz7sensors2v815RenderingSensor6RenderEv");
extern "C" void hook_J(void*self){using F=void(*)(void*);static auto f=(F)actual("_ZN2gz7sensors2v815RenderingSensor6RenderEv","/opt/ros/jazzy/opt/gz_sensors_vendor/lib/libgz-sensors8-rendering.so");boundary_stack('J');auto s=ns(CLOCK_MONOTONIC),c=ns(CLOCK_THREAD_CPUTIME_ID);f(self);record('J',active_source,s,ns(CLOCK_MONOTONIC),ns(CLOCK_THREAD_CPUTIME_ID)-c,1);}

extern "C" void hook_K(void*) asm("_ZN2gz9rendering2v811Ogre2Camera6RenderEv");
extern "C" void hook_K(void*self){using F=void(*)(void*);static auto f=(F)actual("_ZN2gz9rendering2v811Ogre2Camera6RenderEv","/opt/ros/jazzy/opt/gz_rendering_vendor/lib/libgz-rendering8-ogre2.so");boundary_stack('K');auto s=ns(CLOCK_MONOTONIC),c=ns(CLOCK_THREAD_CPUTIME_ID);f(self);record('K',active_source,s,ns(CLOCK_MONOTONIC),ns(CLOCK_THREAD_CPUTIME_ID)-c,1);}

static void gl_identity(){
 using F=const unsigned char*(*)(unsigned); static bool done=false; if(done)return; void*h=dlopen("libGL.so.1",RTLD_NOW|RTLD_NOLOAD); if(!h)h=dlopen("libOpenGL.so.0",RTLD_NOW);auto f=h?(F)dlsym(h,"glGetString"):nullptr;if(!f)return;auto renderer=f(0x1F01);if(!renderer)return;done=true;int fd=open("/tmp/render_gl_identity.txt",O_WRONLY|O_CREAT|O_EXCL,0644);if(fd>=0){dprintf(fd,"host_monotonic_ns=%lld\nvendor=%s\nrenderer=%s\nversion=%s\n",ns(CLOCK_MONOTONIC),f(0x1F00),renderer,f(0x1F02));close(fd);}
}
extern "C" void hook_k(void*) asm("_ZTv0_n696_N2gz9rendering2v811Ogre2Camera6RenderEv");
extern "C" void hook_k(void*self){using F=void(*)(void*);static auto f=(F)actual("_ZTv0_n696_N2gz9rendering2v811Ogre2Camera6RenderEv","/opt/ros/jazzy/opt/gz_rendering_vendor/lib/libgz-rendering8-ogre2.so");boundary_stack('k');auto s=ns(CLOCK_MONOTONIC),c=ns(CLOCK_THREAD_CPUTIME_ID);f(self);gl_identity();record('k',active_source,s,ns(CLOCK_MONOTONIC),ns(CLOCK_THREAD_CPUTIME_ID)-c,1);}

extern "C" void hook_Q(void*) asm("_ZN2gz9rendering2v810Ogre2Scene9PreRenderEv");
extern "C" void hook_Q(void*self){using F=void(*)(void*);static auto f=(F)actual("_ZN2gz9rendering2v810Ogre2Scene9PreRenderEv","/opt/ros/jazzy/opt/gz_rendering_vendor/lib/libgz-rendering8-ogre2.so");boundary_stack('Q');auto s=ns(CLOCK_MONOTONIC),c=ns(CLOCK_THREAD_CPUTIME_ID);f(self);record('Q',active_source,s,ns(CLOCK_MONOTONIC),ns(CLOCK_THREAD_CPUTIME_ID)-c,1);}

extern "C" void hook_q(void*) asm("_ZTv0_n1568_N2gz9rendering2v810Ogre2Scene9PreRenderEv");
extern "C" void hook_q(void*self){using F=void(*)(void*);static auto f=(F)actual("_ZTv0_n1568_N2gz9rendering2v810Ogre2Scene9PreRenderEv","/opt/ros/jazzy/opt/gz_rendering_vendor/lib/libgz-rendering8-ogre2.so");boundary_stack('q');auto s=ns(CLOCK_MONOTONIC),c=ns(CLOCK_THREAD_CPUTIME_ID);f(self);record('q',active_source,s,ns(CLOCK_MONOTONIC),ns(CLOCK_THREAD_CPUTIME_ID)-c,1);}

extern "C" void hook_O(void*) asm("_ZN2gz9rendering2v817Ogre2RenderTarget10PostRenderEv");
extern "C" void hook_O(void*self){using F=void(*)(void*);static auto f=(F)actual("_ZN2gz9rendering2v817Ogre2RenderTarget10PostRenderEv","/opt/ros/jazzy/opt/gz_rendering_vendor/lib/libgz-rendering8-ogre2.so");boundary_stack('O');auto s=ns(CLOCK_MONOTONIC),c=ns(CLOCK_THREAD_CPUTIME_ID);f(self);record('O',active_source,s,ns(CLOCK_MONOTONIC),ns(CLOCK_THREAD_CPUTIME_ID)-c,1);}

extern "C" void hook_o(void*) asm("_ZTv0_n64_N2gz9rendering2v817Ogre2RenderTarget10PostRenderEv");
extern "C" void hook_o(void*self){using F=void(*)(void*);static auto f=(F)actual("_ZTv0_n64_N2gz9rendering2v817Ogre2RenderTarget10PostRenderEv","/opt/ros/jazzy/opt/gz_rendering_vendor/lib/libgz-rendering8-ogre2.so");boundary_stack('o');auto s=ns(CLOCK_MONOTONIC),c=ns(CLOCK_THREAD_CPUTIME_ID);f(self);record('o',active_source,s,ns(CLOCK_MONOTONIC),ns(CLOCK_THREAD_CPUTIME_ID)-c,1);}

extern "C" void hook_D(void*,void*) asm("_ZNK2gz9rendering2v817Ogre2RenderTarget4CopyERNS1_5ImageE");
extern "C" void hook_D(void*self,void*image){using F=void(*)(void*,void*);static auto f=(F)actual("_ZNK2gz9rendering2v817Ogre2RenderTarget4CopyERNS1_5ImageE","/opt/ros/jazzy/opt/gz_rendering_vendor/lib/libgz-rendering8-ogre2.so");boundary_stack('D');auto s=ns(CLOCK_MONOTONIC),c=ns(CLOCK_THREAD_CPUTIME_ID);f(self,image);record('D',active_source,s,ns(CLOCK_MONOTONIC),ns(CLOCK_THREAD_CPUTIME_ID)-c,1);}

extern "C" void hook_d(void*,void*) asm("_ZTv0_n144_NK2gz9rendering2v817Ogre2RenderTarget4CopyERNS1_5ImageE");
extern "C" void hook_d(void*self,void*image){using F=void(*)(void*,void*);static auto f=(F)actual("_ZTv0_n144_NK2gz9rendering2v817Ogre2RenderTarget4CopyERNS1_5ImageE","/opt/ros/jazzy/opt/gz_rendering_vendor/lib/libgz-rendering8-ogre2.so");boundary_stack('d');auto s=ns(CLOCK_MONOTONIC),c=ns(CLOCK_THREAD_CPUTIME_ID);f(self,image);record('d',active_source,s,ns(CLOCK_MONOTONIC),ns(CLOCK_THREAD_CPUTIME_ID)-c,1);}

extern "C" void hook_W(void*) asm("_ZN4Ogre25GL3PlusAsyncTextureTicket23waitForDownloadToFinishEv");
extern "C" void hook_W(void*self){using F=void(*)(void*);static auto f=(F)actual("_ZN4Ogre25GL3PlusAsyncTextureTicket23waitForDownloadToFinishEv","/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib/OGRE-Next/RenderSystem_GL3Plus.so");boundary_stack('W');auto s=ns(CLOCK_MONOTONIC),c=ns(CLOCK_THREAD_CPUTIME_ID);f(self);record('W',active_source,s,ns(CLOCK_MONOTONIC),ns(CLOCK_THREAD_CPUTIME_ID)-c,1);}

extern "C" bool hook_U(void*) asm("_ZN4Ogre4Root14renderOneFrameEv");
extern "C" bool hook_U(void*self){using F=bool(*)(void*);static auto f=(F)actual("_ZN4Ogre4Root14renderOneFrameEv","/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib/libOgreNextMain.so");boundary_stack('U');auto s=ns(CLOCK_MONOTONIC),c=ns(CLOCK_THREAD_CPUTIME_ID);bool ok=f(self);record('U',active_source,s,ns(CLOCK_MONOTONIC),ns(CLOCK_THREAD_CPUTIME_ID)-c,1);return ok;}

extern "C" void hook_workspace(void*self,bool inside) asm("_ZN4Ogre19CompositorWorkspace7_updateEb");
extern "C" void hook_workspace(void*self,bool inside){using F=void(*)(void*,bool);static auto f=(F)actual("_ZN4Ogre19CompositorWorkspace7_updateEb","/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib/libOgreNextMain.so");auto s=ns(CLOCK_MONOTONIC),c=ns(CLOCK_THREAD_CPUTIME_ID);f(self,inside);record('T',active_source,s,ns(CLOCK_MONOTONIC),ns(CLOCK_THREAD_CPUTIME_ID)-c,1);}

extern "C" void hook_scene(void*self,const void*camera) asm("_ZN4Ogre19CompositorPassScene7executeEPKNS_6CameraE");
extern "C" void hook_scene(void*self,const void*camera){using F=void(*)(void*,const void*);static auto f=(F)actual("_ZN4Ogre19CompositorPassScene7executeEPKNS_6CameraE","/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib/libOgreNextMain.so");auto s=ns(CLOCK_MONOTONIC),c=ns(CLOCK_THREAD_CPUTIME_ID);f(self,camera);record('X',active_source,s,ns(CLOCK_MONOTONIC),ns(CLOCK_THREAD_CPUTIME_ID)-c,1);}

extern "C" void hook_quad(void*self,const void*camera) asm("_ZN4Ogre18CompositorPassQuad7executeEPKNS_6CameraE");
extern "C" void hook_quad(void*self,const void*camera){using F=void(*)(void*,const void*);static auto f=(F)actual("_ZN4Ogre18CompositorPassQuad7executeEPKNS_6CameraE","/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib/libOgreNextMain.so");auto s=ns(CLOCK_MONOTONIC),c=ns(CLOCK_THREAD_CPUTIME_ID);f(self,camera);record('V',active_source,s,ns(CLOCK_MONOTONIC),ns(CLOCK_THREAD_CPUTIME_ID)-c,1);}
