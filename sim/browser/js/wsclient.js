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

  // Streaming data for vehicle track (ideal and inferred)
  var simTrack = [];
  var slamTrack = [];
  var simTrackSvg = scene.append( 'g' )
      .attr( 'class', 'simtrack');
  var slamTrackSvg = scene.append( 'g' )
      .attr( 'class', 'slamtrack');
  var lidarLines = scene.append( 'g' )
      .attr( 'class', 'lidar-lines' );


  function drawWaypoints( data ) {
    scene.selectAll( 'circle' )
      .data( data )
      .enter()
      .append( 'circle' )
      .attr( 'r', 0 ) // px
      .transition()
      .duration( 750 )
      .attr( 'r', 4 ) // px
      .attr( 'cx', function( d ) { return xscale( d.x ); })
      .attr( 'cy', function( d ) { return yscale( d.y ); })
      .attr( 'class', 'waypoints' );
  }

  function drawLandmarks( data ) {

    var l = 10; // Edge length of square

    var landmarks = scene.selectAll( 'g' )
      .data( data )
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
      .attr( 'rx', l/8)
      .attr( 'ry', l/8)
      .attr( 'class', 'landmarks' );
  }


  function drawSimTrack( data ) {

    simTrack.push( {
      'x': xscale( data.ideal.x ),
      'y': yscale( data.ideal.y )
    });

    slamTrack.push( {
      'x': xscale( data.slam.x ),
      'y': yscale( data.slam.y )
    });

    var n = simTrack.length;

    if ( n > 1 ) {
      simTrackSvg
        .append( 'line' )
        .attr( 'x1', simTrack[ n - 2 ].x )
        .attr( 'y1', simTrack[ n - 2 ].y )
        .attr( 'x2', simTrack[ n - 1 ].x )
        .attr( 'y2', simTrack[ n - 1 ].y );

      slamTrackSvg
        .append( 'line' )
        .attr( 'x1', slamTrack[ n - 2 ].x )
        .attr( 'y1', slamTrack[ n - 2 ].y )
        .attr( 'x2', slamTrack[ n - 1 ].x )
        .attr( 'y2', slamTrack[ n - 1 ].y );
    }
  }

  function drawLidar( data ) {
    var i = -1;
    while ( ++i < data.length ) {
      scene.selectAll( '.lidar-lines' )
        .append( 'line' )
        .attr( 'x1', xscale( data[ i ].x1 ) )
        .attr( 'y1', yscale( data[ i ].y1 ) )
        .attr( 'x2', xscale( data[ i ].x2 ) )
        .attr( 'y2', yscale( data[ i ].y2 ) )
        .transition()
        .duration( 500 )
        .style( 'opacity', 0 )
        .each( 'end', function() {
          d3.select( this )
            .remove();
        });
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
      case 'tracks':
        drawSimTrack( msg.data );
        break;
      case 'state':
        // TODO
        break;
      case 'lidar':
        drawLidar( msg.data );
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

  // Rotate and translate from a local frame to a global one.
  // Inputs l and g are objects with attributes x, y, and phi.
  function localToGlobal( l, g ) {

    var c = Math.cos(g.phi);
    var s = Math.sin(g.phi);

    var p = l.phi + g.phi;

    if ( p > Math.PI )
      p -= 2*Math.PI;
    if ( p < -Math.PI )
      p += 2*Math.PI;

    return {
      x: c*l.x - s*l.y + g.x,
      y: s*l.x + c*l.y + g.y,
      phi: p
    }
  }

});
