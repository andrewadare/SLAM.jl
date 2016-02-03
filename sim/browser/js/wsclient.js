$( function() {

  // Initiate a WebSocket connection
  var ws = new WebSocket( 'ws://' + window.location.host );

  // Set up the scene SVG element ASAP to avoid reflowing
  var width = 800;
  var height = 800;
  var xscale = d3.scale.linear()
      .domain([ 0, 100 ])
      .range([ 0, width ]);
  var yscale = d3.scale.linear()
      .domain([ 0, 100 ])
      .range([ height, 0 ]);
  var scene = d3.select( '.scene' )
      .attr( 'width', width )
      .attr( 'height', height );

  // Streaming data for simulated vehicle track
  var simTrack = [];
  var simTrackSvg = scene.append( 'g' )
      .attr( 'class', 'simtrack');

  // var line = d3.svg.line()
  //     .x(function( d ) { return xscale( d.x ); })
  //     .y(function( d ) { return yscale( d.y ); })
  //     .interpolate( 'linear' );

  function drawWaypoints( data ) {
    scene.selectAll( 'circle' )
      .data( data )
      .enter()
      .append( 'circle' )
      .attr( 'r', 0 ) // px
      .transition()
      .duration( 750 )
      .attr( 'r', 3 ) // px
      .attr( 'cx', function( d ) { return xscale( d.x ); })
      .attr( 'cy', function( d ) { return yscale( d.y ); })
      .attr( 'class', 'waypoints' );
  }

  function drawLandmarks( data ) {

    var l = 10; // Edge length of square

    var landmarks = scene.selectAll( 'g' )
      .data(data)
      .enter()
      .append( 'g' )
      .attr( 'transform', function( d ) {
        return 'translate(' + (xscale( d.x ) - l/2) + ',' + (yscale( d.y ) - l/2) + ')';
      });

    landmarks.append( 'rect' )
      .attr( 'width', 0 )
      .attr( 'height', 0 )
      .transition()
      .duration( 750 )
      .attr( 'width', l )
      .attr( 'height', l )
      .attr( 'rx', l/5)
      .attr( 'ry', l/5)
      .attr( 'class', 'landmarks' );
  }


  function drawSimTrack( data ) {

    simTrack.push( { 'x': +data[0], 'y': +data[1] });
    var n = simTrack.length;

    if ( n > 1 ) {
      simTrackSvg
        .append( 'line' )
        .attr( 'x1', xscale( simTrack[ n - 2 ].x ))
        .attr( 'y1', yscale( simTrack[ n - 2 ].y ))
        .attr( 'x2', xscale( simTrack[ n - 1 ].x ))
        .attr( 'y2', yscale( simTrack[ n - 1 ].y ));
    }
  }

  ws.onopen = function( event ) {
    ws.send( JSON.stringify( {
      type: 'update',
      text: 'ready',
      id:   0,
      date: Date.now()
    }));

  }

  // Handler for messages received from server
  // https://developer.mozilla.org/en-US/docs/Web/API/WebSockets_API/Writing_WebSocket_client_applications
  ws.onmessage = function( event ) {
    var msg = JSON.parse( event.data );
    switch( msg.type ) {
      case 'waypoints':
        drawWaypoints( msg.data );
        break;
      case 'landmarks':
        drawLandmarks( msg.data );
        break;
      case 'test':
        drawSimTrack( msg.data );
        break;
    }
  }

  $( '#start' ).click( function() {
  
    // Send message object as a JSON-formatted string.
    ws.send( JSON.stringify( {
      type: 'request',
      text: 'start',
      id:   1,
      date: Date.now()
    }));

  });


});
