.. _readers.pcd:

readers.pcd
===========

The **PCD Reader** supports reading from `Point Cloud Data (PCD)`_ formatted
files, which are used by the `Point Cloud Library (PCL)`_.

Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="writers.text">
      <Option name="filename">outputfile.txt</Option>
      <Reader type="readers.pcd">
        <Option name="filename">inputfile.pcd</Option>
      </Reader>
    </Writer>
  </Pipeline>

Options
-------

filename
  PCD file to read [Required] 



.. _Point Cloud Data (PCD): http://pointclouds.org/documentation/tutorials/pcd_file_format.php
.. _Point Cloud Library (PCL): http://pointclouds.org
  
