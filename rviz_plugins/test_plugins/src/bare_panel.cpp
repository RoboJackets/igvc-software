#include <stdio.h>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>

#include "bare_panel.h"

namespace test_plugins
{

BarePanel::BarePanel( QWidget* parent )
  : rviz::Panel( parent )
{
  QHBoxLayout* topic_layout = new QHBoxLayout;
  topic_layout->addWidget( new QLabel( "Output Topic:" ) );
  output_topic_editor_ = new QLineEdit;
  topic_layout->addWidget( output_topic_editor_ );

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout( topic_layout );
  setLayout( layout );

  //connect( output_topic_editor_, SIGNAL( editingFinished() ), this, SLOT( updateTopic() ));
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(test_plugins::BarePanel,rviz::Panel)