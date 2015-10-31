#ifndef QDOUBLESLIDER_H
#define QDOUBLESLIDER_H

#include <QSlider>
namespace reco{
namespace stereo_workbench{


class QDoubleSlider : public QSlider
{
  Q_OBJECT
public:
  explicit QDoubleSlider(QWidget *parent = 0);

  double doubleMinimum(){
    return m_DoubleMin;
  }

  double doubleMaximum()
  {
    return m_DoubleMax;
  }

  double doubleSingleStep()
  {
    return m_DoubleStep;
  }

  void setDoubleMinimum(double value)
  {
    m_DoubleMin = value;
    updateRange();
  }

  void setDoubleMaximum(double value)
  {
    m_DoubleMax = value;
    updateRange();
  }

  void setDoubleSingleStep(double value)
  {
    m_DoubleStep = value;
    updateRange();
  }

  double doubleValue();



signals:
	void doubleValueChanged(double x);

public slots:
	void setDoubleValue(double x);

private:
  double m_DoubleMin;
  double m_DoubleMax;
  double m_DoubleStep;
  double m_DoubleValue;

  int m_CorrespondingIntValue;

  void updateRange();

  void updateStep()
  {
    QSlider::setSingleStep((int)(1000 * m_DoubleStep / (m_DoubleMax - m_DoubleMin)));
  }
};

#endif // QDOUBLESLIDER_H
}//stereo_workbench
}//reco
