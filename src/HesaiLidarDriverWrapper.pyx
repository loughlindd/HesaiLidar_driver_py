# distutils: language = c++
# cython: language_level=3

# robosense_driver.pyx
from libcpp.string cimport string
from libc.stdlib cimport malloc, free
cimport cython
import numpy as np
cimport numpy as cnp

cdef extern from "HesaiDriver.hpp":
    # Expose the LidarStatus struct to Cython
    cdef cppclass LidarStatus:
        LidarStatus() except +
        float temperature
        string sn
        string mac
        string top_fw_ver
        string bottom_fw_ver
        float voltage
    
    cdef cppclass PointCloudFetchResult:
        PointCloudFetchResult() except +
        bint success
        float dt_wait
        float dt_proc
        double timestamp

    cdef cppclass HesaiDriver:
        HesaiDriver() except +
        bint init(int lidar_type,
                  string lidar_address,
                  string host_address,
                  int scans_port,
                  int ptc_port,
                  int fault_message_port,
                  string logging_dir,
                  string log_server_ip,
                  int log_server_port
                  ) except +
        bint start()
        LidarStatus getStatus() const
        void close()
        PointCloudFetchResult getLatestFrame(double* out_buf) except +
        void updateSnapshotDirectory(string snapshot_dir) except +

cdef class PyHesaiDriver:
    cdef HesaiDriver* cpp_driver

    def __cinit__(self):
        self.cpp_driver = new HesaiDriver()

    def __dealloc__(self):
        if self.cpp_driver is not NULL:
            self.cpp_driver.close()
            del self.cpp_driver
            self.cpp_driver = NULL

    def init(
        self, 
        int lidar_type,
        lidar_address,
        host_address,
        int scans_port, 
        int ptc_port, 
        int fault_message_port,
        logging_dir,
        log_server_ip,
        int log_server_port
    ):
        if self.cpp_driver is NULL:
            self.cpp_driver = new HesaiDriver()
        cdef string log_dir_cpp = logging_dir.encode('utf-8') if logging_dir is not None else "".encode("utf-8")
        cdef string host_address_cpp = host_address.encode('utf-8')
        cdef string lidar_address_cpp = lidar_address.encode('utf-8')
        cdef string log_server_ip_cpp = log_server_ip.encode('utf-8') if log_server_ip is not None else "".encode("utf-8")
        cdef int log_server_port_cpp = log_server_port if log_server_port is not None else 0
        return self.cpp_driver.init(lidar_type, lidar_address_cpp, host_address_cpp, scans_port, ptc_port, fault_message_port, log_dir_cpp, log_server_ip_cpp, log_server_port_cpp)

    def start(self):
        return self.cpp_driver.start()

    def getLatestFrame(self, out_arr):
        cdef cnp.ndarray[cnp.float64_t, ndim=2] arr = out_arr
        if not arr.flags['C_CONTIGUOUS']:
            raise ValueError("out_arr must be C-contiguous")
        cdef double* ptr = <double*> arr.data
        cdef PointCloudFetchResult result = self.cpp_driver.getLatestFrame(ptr)
        return result.success, result.dt_wait, result.dt_proc, result.timestamp

    def updateSnapshotDirectory(self, snapshot_dir):
        cdef string snap_dir_cpp = snapshot_dir.encode('utf-8') if snapshot_dir is not None else "".encode("utf-8")
        self.cpp_driver.updateSnapshotDirectory(snap_dir_cpp)

    def getStatus(self):
        """
        Return status as a Python tuple:
        (temperature: float, voltage: float, sn: str, mac: str, top_fw_ver: str, bottom_fw_ver: str, state: bool)
        """
        cdef LidarStatus s = self.cpp_driver.getStatus()
        # Convert std::string fields via their c_str() to Python bytes then decode
        cdef const char* p

        p = s.sn.c_str()
        py_sn = bytes(p).decode('utf-8', 'ignore')

        p = s.mac.c_str()
        py_mac = bytes(p).decode('utf-8', 'ignore')

        p = s.top_fw_ver.c_str()
        py_top = bytes(p).decode('utf-8', 'ignore')

        p = s.bottom_fw_ver.c_str()
        py_bottom = bytes(p).decode('utf-8', 'ignore')

        return (float(s.temperature),
                float(s.voltage),
                py_sn,
                py_mac,
                py_top,
                py_bottom)

    def close(self):
        if self.cpp_driver is not NULL:
            self.cpp_driver.close()
            del self.cpp_driver
            self.cpp_driver = NULL
