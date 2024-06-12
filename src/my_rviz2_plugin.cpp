#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rviz_common/panel.hpp>
#include <QVBoxLayout>
#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QPainter>
#include <QStyleOptionButton>
#include <QRegion>
#include <QPainterPath>
#include <QDebug>

#include <pluginlib/class_list_macros.hpp>

// 円形ボタンを定義するクラス
class CircleButton : public QPushButton
{
public:
  CircleButton(QWidget *parent = nullptr) : QPushButton(parent) 
  {
    setAutoFillBackground(true);
    setPalette(QPalette(Qt::black));
  }

protected:
  void resizeEvent(QResizeEvent *event) override
  {
    QPushButton::resizeEvent(event);
    // ボタンを円形にするためのマスクを設定
    setMask(QRegion(QRect(0, 0, width(), height()), QRegion::Ellipse));
  }

  void paintEvent(QPaintEvent *event) override
  {
    QPainter painter(this);
    QStyleOptionButton option;
    option.initFrom(this);
    style()->drawPrimitive(QStyle::PE_Widget, &option, &painter, this);

    // 円形を描画
    painter.setBrush(palette().button());
    painter.setPen(Qt::NoPen);
    painter.drawEllipse(0, 0, width(), height());
  }

  bool hitButton(const QPoint &pos) const override
  {
    // ボタンのクリック判定を円形にする
    QRegion region(QRect(0, 0, width(), height()), QRegion::Ellipse);
    return region.contains(pos);
  }
};

// SignalPanelクラスの定義
class SignalPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  SignalPanel(QWidget *parent = 0) : rviz_common::Panel(parent)
  {
    // UIのセットアップ
    QVBoxLayout *main_layout = new QVBoxLayout;
    const QStringList labels = {"横方向", "縦方向", "ヨー角"};
    const QStringList colors = {"green", "purple", "yellow", "pink"};
    for (int i = 0; i < 3; ++i)
    {
      QHBoxLayout *row = new QHBoxLayout;
      row->setSpacing(5); // ラベルとボタンの隙間を狭くする
      QLabel *label = new QLabel(labels[i], this);
      row->addWidget(label);
      for (int j = 0; j < 4; ++j)
      {
        CircleButton *button = new CircleButton(this);
        button->setFixedSize(50, 50);
        button->setCheckable(true);
        connect(button, &QPushButton::toggled, [button, colors, j](bool checked) {
          QPalette pal = button->palette();
          pal.setColor(QPalette::Button, checked ? QColor(colors[j]) : Qt::black);
          button->setPalette(pal);
          button->update();
        });
        row->addWidget(button);
        buttons_.push_back(button);
      }
      main_layout->addLayout(row);
    }
    setLayout(main_layout);

    // ROS2のセットアップ
    node_ = std::make_shared<rclcpp::Node>("signal_panel_node");
    subscription_ = node_->create_subscription<std_msgs::msg::String>(
        "/signals", 10, std::bind(&SignalPanel::callback, this, std::placeholders::_1));

    // スピンを開始
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &SignalPanel::spin_node);
    timer_->start(100); // 100ミリ秒ごとにスピン
  }

private slots:
  void spin_node()
  {
    rclcpp::spin_some(node_);
  }

  void callback(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string &data = msg->data;
    for (size_t i = 0; i < data.size() && i < buttons_.size(); ++i)
    {
      buttons_[i]->setChecked(data[i] == '1');
    }
  }

private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  QTimer *timer_;
  std::vector<QPushButton *> buttons_;
};

PLUGINLIB_EXPORT_CLASS(SignalPanel, rviz_common::Panel)

#include "my_rviz2_plugin.moc"