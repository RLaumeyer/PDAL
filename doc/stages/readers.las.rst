.. _readers.las:

readers.las
===========

The **LAS Reader** supports reading from `LAS format`_ files, the standard interchange format for LIDAR data.

Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="writers.text">
      <Option name="filename">outputfile.txt</Option>
      <Reader type="readers.las">
        <Option name="filename">inputfile.las</Option>
      </Reader>
    </Writer>
  </Pipeline>

Options
-------

filename
  LAS file to read [Required] 

extra_dims
  Extra dimensions to be read as part of each point beyond those specified by
  the LAS point format.  The format of the option is
  <dimension_name>=<type>, ... where type is one of:
      int8, int16, int32, int64, uint8, uint16, uint32, uint64, float, double
  '_t' may be added to any of the type names as well (e.g., uint32_t)

.. _LAS format: http://asprs.org/Committee-General/LASer-LAS-File-Format-Exchange-Activities.html
  
