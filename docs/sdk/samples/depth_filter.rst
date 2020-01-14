.. _depth_filter:

User filter to filter deep data
====================================

Filter type is inherited from ``BaseFilter`` .

Method port protocol is as follows：

.. code-block:: c++

    virtual bool ProcessFrame(
          std::shared_ptr<Image> out,
          const std::shared_ptr<Image> in) = 0; // NOLINT
    virtual bool LoadConfig(void* data);

    inline bool TurnOn();
    inline bool TurnOff();
    inline bool IsEnable();


    int main(int argc, char const* argv[]) {
    ...

    SpatialFilter spat_filter;
    TemporalFilter temp_filter;

    ...
    for (;;) {
      // get frame
      ...
      spat_filter.ProcessFrame(image_depth.img, image_depth.img);
      temp_filter.ProcessFrame(image_depth.img, image_depth.img);
      ...
    }


.. tip::

    When using, instantiate a ``Filter`` ,then use it directly in the image processing loop ``ProcessFrame`` .
    The image will adapt to the image information in real time. You can also use the ``TurnOn/TurnOff`` switch in real time.
