#include "pserial.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#ifdef POSIX_SERIAL_COMM_CWRAP_DEBUG
#define DEBUG
#endif
#include "debug.h"

#define MAX_HANDLE 100
extern "C" {

  static common::serial::AbstractSerialComm* id2handle[MAX_HANDLE]={0};

  pserial_handle_t popen_serial(int internal_buffering,const char*serdev,int baudrate,int parity,int bits,int stop){
    int idx=-1;
    if(serdev){
      
      char *pnt;
      int len = strlen(serdev);
      pnt = (char*)serdev+len-1;
      while(isdigit(*pnt) && pnt>serdev){
	pnt --;
      }
      if(pnt == (serdev+len)){
	printf("## bad serial number specification %s\n",serdev);
	return -1;
      }
      idx = atoi(pnt+1);
    }

    if(idx<0 || idx>=MAX_HANDLE){
      printf("## bad index specification %d\n",idx);
      return -1;
    }
    if(id2handle[idx]!=0){
      // close before
      printf("%% serial resource %d was open, closing\n",idx);
      delete id2handle[idx];
      id2handle[idx]=0;
    }
        common::serial::AbstractSerialComm*p = new common::serial::PosixSerialComm(serdev,baudrate,parity,bits,stop,internal_buffering,internal_buffering);
    //    common::serial::AbstractSerialComm *p = new common::serial::PosixSerialCommSimple(serdev,baudrate,parity,bits,stop);
    if(p){
      if(p->init()!=0){
	printf("## error during initialization\n");
	delete p;
	return -7;
      } else {
	id2handle[idx] = p;
	DPRINT("opening serial \"%d\" = 0x%x\n",idx,p);
	return idx;
      }
    }
    return idx;
  }


  int pclose_serial(pserial_handle_t _h){
    int ret=-1;
    if(_h>=MAX_HANDLE || _h<0) {
      printf("## bad handle\n");
      return -4;
    }
    common::serial::AbstractSerialComm* h = id2handle[_h];
    if(h){
      common::serial::AbstractSerialComm *p =( common::serial::AbstractSerialComm *)h;
      ret = p->deinit();
      delete p;
      id2handle[_h] = 0;
      return ret;
    }
    return ret;
  }

  int pwrite_async_serial(pserial_handle_t _h, char*buf,int bsize){
    int ret=-2;
    if(_h>=MAX_HANDLE || _h<0) {
      printf("## bad handle\n");
      return -4;
    }
    common::serial::AbstractSerialComm* h = id2handle[_h];
    if(h){

      common::serial::AbstractSerialComm*p =( common::serial::AbstractSerialComm*)h;
      ret = p->write_async((void*)buf,bsize);
    }
    return ret;
  }

  int pread_async_serial(pserial_handle_t _h, char*buf,int bsize){
    int ret=-2;
    if(_h>=MAX_HANDLE || _h<0) {
      printf("## bad handle\n");
      return -4;
    }
    common::serial::AbstractSerialComm* h = id2handle[_h];
    if(h){
      common::serial::AbstractSerialComm*p =( common::serial::AbstractSerialComm*)h;
      ret = p->read_async((void*)buf,bsize);
    }
    return ret;

  }
  int pwrite_serial(pserial_handle_t _h, char*buf,int bsize,int timeo,int*timocc){
    int ret=-2;
     if(_h>=MAX_HANDLE || _h<0) {
      printf("## bad handle\n");
      return -4;
    }
    common::serial::AbstractSerialComm* h = id2handle[_h];
    if(h){
      DPRINT("buf 0x%x size %d timeo %d\n",buf,bsize,timeo);
      common::serial::AbstractSerialComm*p =( common::serial::AbstractSerialComm*)h;
      ret = p->write((void*)buf,bsize,timeo,timocc);
      DPRINT("done return %d timeo %d\n",ret,*timocc);
    }
    return ret;

  }
  int pread_serial(pserial_handle_t _h, char*buf,int bsize,int timeo,int*timocc){
    int ret=-2;
    if(_h>=MAX_HANDLE || _h<0) {
      printf("## bad handle\n");
      return -4;
    }
    common::serial::AbstractSerialComm* h = id2handle[_h];
    
    if(h){
      common::serial::AbstractSerialComm*p =( common::serial::AbstractSerialComm*)h;
      DPRINT("buf 0x%x size %d timeo %d\n",buf,bsize,timeo);
      ret = p->read((void*)buf,bsize,timeo,timocc);
      DPRINT("done return %d timeo %d\n",ret,*timocc);
    }
    return ret;

  }
  int byte_available_read_serial(pserial_handle_t _h){
    int ret=0;
    if(_h>=MAX_HANDLE || _h<0) {
      printf("## bad handle\n");
      return -4;
    }
    common::serial::AbstractSerialComm* h = id2handle[_h];
    if(h){

      common::serial::AbstractSerialComm*p =( common::serial::AbstractSerialComm*)h;
      ret = p->byte_available_read();
    }
    return ret;

  }

  int byte_available_write_serial(pserial_handle_t _h){
    int ret=0;
    if(_h>=MAX_HANDLE || _h<0) {
      printf("## bad handle\n");
      return -4;
    }
    common::serial::AbstractSerialComm* h = id2handle[_h];
    if(h){
      common::serial::AbstractSerialComm*p =( common::serial::AbstractSerialComm*)h;
      ret = p->byte_available_write();
    }
    return ret;

  }
  void pwrite_flush_serial(pserial_handle_t _h){
    if(_h>=MAX_HANDLE || _h<0) {
      printf("## bad handle\n");
      return ;
    }
    common::serial::AbstractSerialComm* h = id2handle[_h];
    if(h){
      common::serial::AbstractSerialComm*p =( common::serial::AbstractSerialComm*)h;
      p->flush_write();
    }
  }
  void pread_flush_serial(pserial_handle_t _h){
    if(_h>=MAX_HANDLE || _h<0) {
      printf("## bad handle\n");
      return ;
    }
    common::serial::AbstractSerialComm* h = id2handle[_h];

    if(h){
      common::serial::AbstractSerialComm*p =( common::serial::AbstractSerialComm*)h;
      p->flush_read();
    }

  }


  int LVwrite_serial(pserial_handle_t h,void*buf,int bytes,int timeo,int*timoccur){
    int ret=-1;
    DPRINT("WRITE resource %d(0x%x), buf: @x%x size %d timeo %d timeout occur(%d) x%x\n",h, id2handle[h],buf,bytes,timeo,*timoccur,timoccur);

    if(timeo>0){
      ret = pwrite_serial(h, (char*)buf,bytes,timeo,timoccur); 
    } else if(timeo==0){
      ret = pwrite_async_serial(h, (char*)buf,bytes); 
    } else {
      ret = pwrite_serial(h, (char*)buf,bytes,0,timoccur); 
    }
    return ret;
}

  int LVread_serial(pserial_handle_t h,void*buf,int bytes,int timeo,int*timoccur){
    int ret=-1;
    DPRINT(" READ resource %d(0x%x), buf: @x%x size %d timeo %d timeout occur(%d) x%x\n",h,id2handle[h],buf,bytes,timeo,*timoccur,timoccur);
    if(timeo>0){
      ret = pread_serial(h, (char*)buf,bytes,timeo,timoccur); 
    } else if(timeo==0){
      ret = pread_async_serial(h, (char*)buf,bytes); 
    } else {
      ret = pread_serial(h, (char*)buf,bytes,0,timoccur); 
    }
    return ret;
  }

}
